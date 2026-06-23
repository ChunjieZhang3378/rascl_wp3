#include "rascl_hardware_interface/rascl_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "soem/soem.h"

namespace rascl_hardware_interface
{

namespace
{
// CiA 402 object dictionary entries accessed through CoE SDO communication.
constexpr uint16_t kControlWord = 0x6040;
constexpr uint16_t kStatusWord = 0x6041;
constexpr uint16_t kOperationMode = 0x6060;
constexpr uint16_t kOperationModeDisplay = 0x6061;
constexpr uint16_t kPositionActualValue = 0x6064;
constexpr uint16_t kTargetPosition = 0x607A;
constexpr uint16_t kHomingMethod = 0x6098;
constexpr uint16_t kRxPdoMapping = 0x1600;
constexpr uint16_t kTxPdoMapping = 0x1A00;
constexpr uint16_t kRxPdoAssignment = 0x1C12;
constexpr uint16_t kTxPdoAssignment = 0x1C13;

constexpr uint32_t kRxPdoControlWord = 0x60400010;
constexpr uint32_t kRxPdoTargetPosition = 0x607A0020;
constexpr uint32_t kRxPdoOperationMode = 0x60600008;
constexpr uint32_t kTxPdoStatusWord = 0x60410010;
constexpr uint32_t kTxPdoActualPosition = 0x60640020;
constexpr uint32_t kTxPdoOperationModeDisplay = 0x60610008;
constexpr size_t kCspPdoSize = 7;
constexpr size_t kIoMapSize = 4096;

constexpr int8_t kHomingMode = 6;
constexpr int8_t kCyclicSynchronousPositionMode = 8;
constexpr int8_t kHomingMethodRisingEdgeTop = 28;
constexpr int8_t kHomingMethodEndEffector = 37;
constexpr uint16_t kControlEnableVoltage = 0x0006;
constexpr uint16_t kControlSwitchOn = 0x0007;
constexpr uint16_t kControlEnableOperation = 0x000F;
constexpr uint16_t kControlFaultReset = 0x0080;
constexpr uint16_t kControlStartMotion = 0x001F;
constexpr uint16_t kControlDisableVoltage = 0x0000;
constexpr uint16_t kStatusFault = 0x0008;
constexpr uint16_t kStatusHomingAttained = 0x1000;
constexpr uint16_t kStatusHomingError = 0x2000;
constexpr auto kHomingTimeout = std::chrono::seconds(30);
constexpr auto kHomingPollInterval = std::chrono::milliseconds(100);

const rclcpp::Logger kLogger = rclcpp::get_logger("rascl_hardware_interface");
}  // namespace

hardware_interface::CallbackReturn RasclHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto joint_count = info_.joints.size();

  // Allocate once during initialization so the cyclic path does not resize.
  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_actual_positions_.assign(joint_count, 0.0);
  hw_status_words_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  lower_position_limits_.assign(joint_count, -std::numeric_limits<double>::infinity());
  upper_position_limits_.assign(joint_count, std::numeric_limits<double>::infinity());
  command_limit_warning_active_.assign(joint_count, false);
  slave_ids_.resize(joint_count);
  drive_units_per_radian_.assign(joint_count, 1.0);
  last_status_words_.assign(joint_count, 0);

  // SOEM opens a raw Ethernet socket by interface name, so the adapter must
  // match a network interface visible inside the container.
  const auto adapter = info_.hardware_parameters.find("adapter");
  if (adapter == info_.hardware_parameters.end() || adapter->second.empty()) {
    RCLCPP_ERROR(kLogger, "Missing required hardware parameter 'adapter'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  adapter_ = adapter->second;

  // Parse all joint-specific values once.
  for (auto i = 0u; i < joint_count; ++i) {
    const auto & joint = info_.joints[i];

    // Default to EtherCAT bus order when no explicit slave ID is supplied.
    slave_ids_[i] = static_cast<uint16_t>(i + 1);
    const auto slave_id = joint.parameters.find("slave_id");
    if (slave_id != joint.parameters.end()) {
      slave_ids_[i] = static_cast<uint16_t>(std::stoi(slave_id->second));
    }

    // Convert between ROS radians and the position counts used by the drive.
    const auto scale = joint.parameters.find("drive_units_per_radian");
    if (scale == joint.parameters.end()) {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' is missing 'drive_units_per_radian'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    drive_units_per_radian_[i] = std::stod(scale->second);

    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' must have one position command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Position limits are optional, but a one-sided interval is invalid.
    const auto & position_command_interface = joint.command_interfaces[0];
    const bool has_lower_limit = !position_command_interface.min.empty();
    const bool has_upper_limit = !position_command_interface.max.empty();
    if (has_lower_limit != has_upper_limit) {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' must define both position limits or neither",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    try {
      if (has_lower_limit) {
        lower_position_limits_[i] = std::stod(position_command_interface.min);
        upper_position_limits_[i] = std::stod(position_command_interface.max);
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' has an invalid position limit: %s",
        joint.name.c_str(), exception.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_lower_limit &&
      (!std::isfinite(lower_position_limits_[i]) ||
      !std::isfinite(upper_position_limits_[i]) ||
      lower_position_limits_[i] > upper_position_limits_[i]))
    {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' must have finite position limits with min <= max",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // ros2_control requires these states for position trajectory feedback.
    bool has_position_state = false;
    bool has_velocity_state = false;
    for (const auto & state_interface : joint.state_interfaces) {
      has_position_state = has_position_state ||
        state_interface.name == hardware_interface::HW_IF_POSITION;
      has_velocity_state = has_velocity_state ||
        state_interface.name == hardware_interface::HW_IF_VELOCITY;
    }

    if (!has_position_state || !has_velocity_state) {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' must expose position and velocity states", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Start command storage at the declared initial position to avoid a
    // jump before the first controller update.
    const auto initial_value = joint.command_interfaces[0].parameters.find("initial_value");
    if (initial_value != joint.command_interfaces[0].parameters.end()) {
      hw_positions_[i] = std::stod(initial_value->second);
      hw_commands_[i] = hw_positions_[i];
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  // Discard any state left from an earlier lifecycle run before reopening SOEM.
  ethercat_context_ = {};
  ethercat_initialized_ = false;
  ethercat_operational_ = false;
  csp_process_data_ready_ = false;

  // Open the raw-socket master on the configured network interface.
  if (!ecx_init(&ethercat_context_, adapter_.c_str())) {
    RCLCPP_ERROR(kLogger, "Failed to initialize SOEM on adapter '%s'", adapter_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Scan the bus, discover slaves, and initialize mailbox communication.
  const int slave_count = ecx_config_init(&ethercat_context_);
  if (slave_count <= 0) {
    RCLCPP_ERROR(kLogger, "No EtherCAT slaves found on adapter '%s'", adapter_.c_str());
    ecx_close(&ethercat_context_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Configure distributed clocks when supported by the connected slaves.
  ecx_configdc(&ethercat_context_);
  ethercat_initialized_ = true;

  // SDO configuration is performed in PRE-OP, before cyclic operation starts.
  for (int slave = 1; slave <= slave_count; ++slave) {
    ethercat_context_.slavelist[slave].state = EC_STATE_PRE_OP;
    ecx_writestate(&ethercat_context_, slave);

    const int reached_state = ecx_statecheck(
      &ethercat_context_, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    if ((reached_state & EC_STATE_PRE_OP) == 0) {
      RCLCPP_ERROR(kLogger, "Slave %d did not reach PRE-OP", slave);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Verify that every URDF joint refers to a slave discovered on this bus.
  for (auto i = 0u; i < slave_ids_.size(); ++i) {
    const auto slave = slave_ids_[i];
    const auto & joint_name = info_.joints[i].name;

    if (slave < 1 || slave > slave_count) {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' slave_id %u is outside detected range 1..%d",
        joint_name.c_str(), slave, slave_count);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Mailbox communication is established by ecx_config_init().
    uint32_t identity_value = 0;
    int size = sizeof(identity_value);
    if (ecx_SDOread(
        &ethercat_context_, slave, 0x1018, 0x00, FALSE, &size, &identity_value,
        EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_WARN(
        kLogger, "Could not read 0x1018 for joint '%s', continuing basic setup",
        joint_name.c_str());
    }
  }

  // Homing uses mailbox SDO access, so do not make PDO/CSP readiness a
  // configuration prerequisite. This mirrors the older configuration flow and
  // allows activation to run the drive-supported homing sequence first.
  const int safe_op_state = ecx_statecheck(
    &ethercat_context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
  if ((safe_op_state & EC_STATE_SAFE_OP) != 0) {
    ethercat_context_.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ethercat_context_, 0);

    const int reached_state = ecx_statecheck(
      &ethercat_context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    if ((reached_state & EC_STATE_OPERATIONAL) == 0) {
      RCLCPP_WARN(kLogger, "EtherCAT slaves did not reach OPERATIONAL; continuing with SDO access");
    }
  } else {
    RCLCPP_WARN(kLogger, "EtherCAT slaves did not reach SAFE-OP; continuing with SDO access");
  }

  ethercat_operational_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (!ethercat_initialized_) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!ethercat_operational_) {
    RCLCPP_ERROR(kLogger, "EtherCAT communication is not ready");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Keep statusword access and masked CiA 402 transition checks consistent
  // throughout the activation sequence.
  auto read_status_word = [this](const uint16_t slave, const std::string & joint_name,
    uint16_t & status_word) {
      int size = sizeof(status_word);
      if (ecx_SDOread(
          &ethercat_context_, slave, kStatusWord, 0x00, FALSE, &size, &status_word,
          EC_TIMEOUTRXM) <= 0)
      {
        RCLCPP_ERROR(kLogger, "Failed to read statusword for joint '%s'", joint_name.c_str());
        return false;
      }
      return true;
    };

  auto wait_for_status = [this, &read_status_word](
    const uint16_t slave, const std::string & joint_name,
    const uint16_t mask, const uint16_t value,
    uint16_t & status_word) {
      for (int attempt = 0; attempt < 50; ++attempt) {
        if (read_status_word(slave, joint_name, status_word) &&
          (status_word & mask) == value)
        {
          return true;
        }
      }
      return false;
    };

  // Stage 1: select Homing mode and bring every drive through the CiA 402
  // state machine to Operation Enabled.
  for (auto i = 0u; i < slave_ids_.size(); ++i) {
    const auto slave = slave_ids_[i];
    const auto & joint_name = info_.joints[i].name;

    uint16_t status_word = 0;

    int8_t mode = kHomingMode;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kOperationMode, 0x00, FALSE, sizeof(mode), &mode,
        EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to set homing mode for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    int8_t mode_display = 0;
    int size = sizeof(mode_display);
    bool confirmed_homing_mode = false;
    for (int attempt = 0; attempt < 50; ++attempt) {
      mode_display = 0;
      size = sizeof(mode_display);
      if (ecx_SDOread(
          &ethercat_context_, slave, kOperationModeDisplay, 0x00, FALSE, &size,
          &mode_display, EC_TIMEOUTRXM) > 0 && mode_display == kHomingMode)
      {
        confirmed_homing_mode = true;
        break;
      }
    }
    if (!confirmed_homing_mode) {
      RCLCPP_WARN(
        kLogger, "Joint '%s' did not confirm mode 0x6061 = 6, continuing activation",
        joint_name.c_str());
    }

    // The arm axes reference a switch edge; the end effector uses its own
    // drive-specific homing strategy.
    int8_t homing_method;

    if (joint_name == "end_effector_joint") {
      homing_method = kHomingMethodEndEffector;
    } else {
      homing_method = kHomingMethodRisingEdgeTop;
    }

    int8_t current_homing_method = 0;
    size = sizeof(current_homing_method);
    const bool read_current_homing_method = ecx_SDOread(
      &ethercat_context_, slave, kHomingMethod, 0x00, FALSE, &size,
      &current_homing_method, EC_TIMEOUTRXM) > 0;
    if (!read_current_homing_method || current_homing_method != homing_method) {
      if (ecx_SDOwrite(
          &ethercat_context_, slave, kHomingMethod, 0x00, FALSE, sizeof(homing_method),
          &homing_method, EC_TIMEOUTRXM) <= 0)
      {
        if (read_current_homing_method) {
          RCLCPP_ERROR(
            kLogger,
            "Failed to set homing method for joint '%s' to %d; drive currently reports %d",
            joint_name.c_str(), homing_method, current_homing_method);
        } else {
          RCLCPP_ERROR(
            kLogger, "Failed to set homing method for joint '%s' to %d and could not read 0x6098",
            joint_name.c_str(), homing_method);
        }
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // If the drive is in fault, send fault reset (0x0080) before normal CiA402 steps.
    size = sizeof(status_word);
    if (ecx_SDOread(
        &ethercat_context_, slave, kStatusWord, 0x00, FALSE, &size, &status_word,
        EC_TIMEOUTRXM) > 0 && (status_word & kStatusFault) != 0)
    {
      uint16_t control_word = kControlFaultReset;
      if (ecx_SDOwrite(
          &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
          &control_word, EC_TIMEOUTRXM) <= 0)
      {
        RCLCPP_ERROR(
          kLogger, "Failed to send 0x0080 fault reset for joint '%s'", joint_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      bool fault_cleared = false;
      for (int attempt = 0; attempt < 50; ++attempt) {
        status_word = 0;
        size = sizeof(status_word);
        if (ecx_SDOread(
            &ethercat_context_, slave, kStatusWord, 0x00, FALSE, &size, &status_word,
            EC_TIMEOUTRXM) > 0 && (status_word & kStatusFault) == 0)
        {
          fault_cleared = true;
          break;
        }
      }
      if (!fault_cleared) {
        RCLCPP_ERROR(kLogger, "Joint '%s' stayed in fault after 0x0080 reset", joint_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Controlword 0x0006 -> Ready to Switch On (0x0021).
    uint16_t control_word = kControlEnableVoltage;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to send 0x0006 for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    status_word = 0;
    if (!wait_for_status(slave, joint_name, 0x006F, 0x0021, status_word)) {
      RCLCPP_ERROR(kLogger, "Joint '%s' did not reach statusword 0x0021", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Controlword 0x0007 -> Switched On (0x0023).
    control_word = kControlSwitchOn;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to send 0x0007 for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!wait_for_status(slave, joint_name, 0x006F, 0x0023, status_word)) {
      RCLCPP_ERROR(kLogger, "Joint '%s' did not reach statusword 0x0023", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Controlword 0x000F -> Operation Enabled (0x0027) before starting homing.
    control_word = kControlEnableOperation;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to send 0x000F for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!wait_for_status(slave, joint_name, 0x006F, 0x0027, status_word)) {
      RCLCPP_ERROR(kLogger, "Joint '%s' did not reach statusword 0x0027", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Stage 2: start all axes before waiting, allowing them to home in parallel.
  for (auto i = 0u; i < slave_ids_.size(); ++i) {
    const auto slave = slave_ids_[i];
    const auto & joint_name = info_.joints[i].name;

    // Controlword bit 4 starts homing with the configured controller homing method.
    uint16_t control_word = kControlEnableOperation;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to clear homing start bit for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    control_word = kControlStartMotion;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to start homing for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Poll each statusword until every axis reports homing attained, one reports
  // a homing error, or the common deadline expires.
  std::vector<bool> homing_attained(slave_ids_.size(), false);
  std::vector<uint16_t> homing_status_words(slave_ids_.size(), 0);
  const auto homing_deadline = std::chrono::steady_clock::now() + kHomingTimeout;
  while (std::chrono::steady_clock::now() < homing_deadline) {
    bool all_homed = true;
    for (auto i = 0u; i < slave_ids_.size(); ++i) {
      if (homing_attained[i]) {
        continue;
      }

      const auto slave = slave_ids_[i];
      const auto & joint_name = info_.joints[i].name;
      uint16_t status_word = 0;
      if (!read_status_word(slave, joint_name, status_word)) {
        all_homed = false;
        continue;
      }
      homing_status_words[i] = status_word;

      if ((status_word & kStatusHomingError) != 0) {
        RCLCPP_ERROR(
          kLogger, "Joint '%s' reported homing error, statusword: 0x%04X",
          joint_name.c_str(), status_word);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((status_word & kStatusHomingAttained) != 0) {
        homing_attained[i] = true;
        RCLCPP_INFO(
          kLogger, "Joint '%s' homing attained, statusword: 0x%04X",
          joint_name.c_str(), status_word);
      } else {
        all_homed = false;
      }
    }

    if (all_homed) {
      break;
    }
    std::this_thread::sleep_for(kHomingPollInterval);
  }

  // Disable any axis that did not home so a failed activation cannot leave it
  // energized indefinitely.
  for (auto i = 0u; i < slave_ids_.size(); ++i) {
    if (homing_attained[i]) {
      continue;
    }

    const auto slave = slave_ids_[i];
    const auto & joint_name = info_.joints[i].name;
    uint16_t control_word = kControlDisableVoltage;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_WARN(
        kLogger, "Failed to disable voltage after homing timeout for joint '%s'",
        joint_name.c_str());
    }
    RCLCPP_ERROR(
      kLogger, "Joint '%s' did not report homing attained within 30 s, statusword: 0x%04X",
      joint_name.c_str(), homing_status_words[i]);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Homing is complete. Now prepare the process-data path required by CSP.
  // PDO remapping is intentionally done after homing so a CSP mapping problem
  // cannot prevent the SDO-based homing sequence from running.
  for (const auto slave : slave_ids_) {
    ethercat_context_.slavelist[slave].state = EC_STATE_PRE_OP;
    ecx_writestate(&ethercat_context_, slave);

    const int reached_state = ecx_statecheck(
      &ethercat_context_, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    if ((reached_state & EC_STATE_PRE_OP) == 0) {
      RCLCPP_ERROR(kLogger, "Slave %u did not return to PRE-OP for CSP PDO setup", slave);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (auto i = 0u; i < slave_ids_.size(); ++i) {
    const auto slave = slave_ids_[i];
    const auto & joint_name = info_.joints[i].name;

    uint8_t number_of_entries = 0;
    uint16_t pdo_mapping = kRxPdoMapping;
    const uint32_t rx_entries[] = {
      kRxPdoControlWord, kRxPdoTargetPosition, kRxPdoOperationMode};

    if (ecx_SDOwrite(
        &ethercat_context_, slave, kRxPdoAssignment, 0x00, FALSE,
        sizeof(number_of_entries), &number_of_entries, EC_TIMEOUTRXM) <= 0 ||
      ecx_SDOwrite(
        &ethercat_context_, slave, kRxPdoMapping, 0x00, FALSE,
        sizeof(number_of_entries), &number_of_entries, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to clear RxPDO mapping for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (uint8_t entry = 0; entry < 3; ++entry) {
      if (ecx_SDOwrite(
          &ethercat_context_, slave, kRxPdoMapping, entry + 1, FALSE,
          sizeof(rx_entries[entry]), &rx_entries[entry], EC_TIMEOUTRXM) <= 0)
      {
        RCLCPP_ERROR(kLogger, "Failed to map RxPDO for joint '%s'", joint_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    number_of_entries = 3;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kRxPdoMapping, 0x00, FALSE,
        sizeof(number_of_entries), &number_of_entries, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to enable RxPDO mapping for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    number_of_entries = 1;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kRxPdoAssignment, 0x01, FALSE,
        sizeof(pdo_mapping), &pdo_mapping, EC_TIMEOUTRXM) <= 0 ||
      ecx_SDOwrite(
        &ethercat_context_, slave, kRxPdoAssignment, 0x00, FALSE,
        sizeof(number_of_entries), &number_of_entries, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to assign RxPDO for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    number_of_entries = 0;
    pdo_mapping = kTxPdoMapping;
    const uint32_t tx_entries[] = {
      kTxPdoStatusWord, kTxPdoActualPosition, kTxPdoOperationModeDisplay};

    if (ecx_SDOwrite(
        &ethercat_context_, slave, kTxPdoAssignment, 0x00, FALSE,
        sizeof(number_of_entries), &number_of_entries, EC_TIMEOUTRXM) <= 0 ||
      ecx_SDOwrite(
        &ethercat_context_, slave, kTxPdoMapping, 0x00, FALSE,
        sizeof(number_of_entries), &number_of_entries, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to clear TxPDO mapping for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (uint8_t entry = 0; entry < 3; ++entry) {
      if (ecx_SDOwrite(
          &ethercat_context_, slave, kTxPdoMapping, entry + 1, FALSE,
          sizeof(tx_entries[entry]), &tx_entries[entry], EC_TIMEOUTRXM) <= 0)
      {
        RCLCPP_ERROR(kLogger, "Failed to map TxPDO for joint '%s'", joint_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    number_of_entries = 3;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kTxPdoMapping, 0x00, FALSE,
        sizeof(number_of_entries), &number_of_entries, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to enable TxPDO mapping for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    number_of_entries = 1;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kTxPdoAssignment, 0x01, FALSE,
        sizeof(pdo_mapping), &pdo_mapping, EC_TIMEOUTRXM) <= 0 ||
      ecx_SDOwrite(
        &ethercat_context_, slave, kTxPdoAssignment, 0x00, FALSE,
        sizeof(number_of_entries), &number_of_entries, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to assign TxPDO for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  io_map_.assign(kIoMapSize, 0);
  const int mapped_bytes = ecx_config_map_group(&ethercat_context_, io_map_.data(), 0);
  if (mapped_bytes <= 0 || static_cast<size_t>(mapped_bytes) > io_map_.size()) {
    RCLCPP_ERROR(kLogger, "Failed to create EtherCAT process-data map for CSP");
    return hardware_interface::CallbackReturn::ERROR;
  }

  ecx_configdc(&ethercat_context_);
  expected_work_counter_ =
    ethercat_context_.grouplist[0].outputsWKC * 2 +
    ethercat_context_.grouplist[0].inputsWKC;

  const int safe_op_state = ecx_statecheck(
    &ethercat_context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
  if ((safe_op_state & EC_STATE_SAFE_OP) == 0) {
    RCLCPP_ERROR(kLogger, "EtherCAT slaves did not reach SAFE-OP for CSP");
    return hardware_interface::CallbackReturn::ERROR;
  }

  ecx_send_processdata(&ethercat_context_);
  ecx_receive_processdata(&ethercat_context_, EC_TIMEOUTRET);
  ethercat_context_.slavelist[0].state = EC_STATE_OPERATIONAL;
  ecx_writestate(&ethercat_context_, 0);

  const int reached_state = ecx_statecheck(
    &ethercat_context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
  if ((reached_state & EC_STATE_OPERATIONAL) == 0) {
    RCLCPP_ERROR(kLogger, "EtherCAT slaves did not reach OPERATIONAL for CSP");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Stage 3: leave Homing mode, select Cyclic Synchronous Position mode, and
  // synchronize the ROS command with the measured post-homing position.
  for (auto i = 0u; i < slave_ids_.size(); ++i) {
    const auto slave = slave_ids_[i];
    const auto & joint_name = info_.joints[i].name;

    int8_t mode = kCyclicSynchronousPositionMode;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kOperationMode, 0x00, FALSE, sizeof(mode), &mode,
        EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(
        kLogger, "Failed to set CSP mode for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    int8_t mode_display = 0;
    int size = sizeof(mode_display);
    bool confirmed_csp_mode = false;
    for (int attempt = 0; attempt < 50; ++attempt) {
      mode_display = 0;
      size = sizeof(mode_display);
      if (ecx_SDOread(
          &ethercat_context_, slave, kOperationModeDisplay, 0x00, FALSE, &size,
          &mode_display, EC_TIMEOUTRXM) > 0 &&
        mode_display == kCyclicSynchronousPositionMode)
      {
        confirmed_csp_mode = true;
        break;
      }
    }
    if (!confirmed_csp_mode) {
      RCLCPP_WARN(
        kLogger, "Joint '%s' did not confirm mode 0x6061 = 8, continuing activation",
        joint_name.c_str());
    }

    uint16_t status_word = 0;

    // PDO remapping can leave the drive back in Switch On Disabled (0x0040).
    // Run the complete CiA 402 enable sequence again before entering CSP.
    uint16_t control_word = kControlEnableVoltage;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to send 0x0006 for joint '%s' after homing", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!wait_for_status(slave, joint_name, 0x006F, 0x0021, status_word)) {
      RCLCPP_WARN(
        kLogger,
        "Joint '%s' did not confirm statusword 0x0021 after homing, continuing; "
        "latest statusword: 0x%04X",
        joint_name.c_str(), status_word);
    }

    control_word = kControlSwitchOn;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(
        kLogger, "Failed to send 0x0007 for joint '%s' after homing", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!wait_for_status(slave, joint_name, 0x006F, 0x0023, status_word)) {
      RCLCPP_WARN(
        kLogger,
        "Joint '%s' did not confirm statusword 0x0023 after homing, continuing; "
        "latest statusword: 0x%04X",
        joint_name.c_str(), status_word);
    }

    int32_t actual_position = 0;
    size = sizeof(actual_position);
    if (ecx_SDOread(
        &ethercat_context_, slave, kPositionActualValue, 0x00, FALSE, &size,
        &actual_position, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(
        kLogger, "Failed to read actual position after homing joint '%s'",
        joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_actual_positions_[i] = static_cast<double>(actual_position);
    hw_positions_[i] = static_cast<double>(actual_position) / drive_units_per_radian_[i];
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = hw_positions_[i];

    // Preload the current position before enabling operation. This prevents the
    // drive from acting on a stale target left from an earlier session.
    const int32_t target_position = actual_position;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kTargetPosition, 0x00, FALSE, sizeof(target_position),
        &target_position, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to preload target position for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Controlword 0x000F -> Operation Enabled (0x0027).
    control_word = kControlEnableOperation;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_ERROR(kLogger, "Failed to send 0x000F for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    status_word = 0;
    if (!wait_for_status(slave, joint_name, 0x006F, 0x0027, status_word)) {
      RCLCPP_WARN(
        kLogger,
        "Joint '%s' did not confirm statusword 0x0027 after CSP setup, continuing; "
        "latest statusword: 0x%04X",
        joint_name.c_str(), status_word);
    } else {
      RCLCPP_INFO(
        kLogger, "Joint '%s' reached operation enabled, statusword: 0x%04X",
        joint_name.c_str(), status_word);
    }

    if (!read_status_word(slave, joint_name, status_word)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      kLogger,
      "Joint '%s' activation complete, final statusword: 0x%04X, current position: %.6f rad",
      joint_name.c_str(), status_word, hw_positions_[i]);

    last_status_words_[i] = status_word;

    uint8_t * outputs = ethercat_context_.slavelist[slave].outputs;
    if (outputs == nullptr || ethercat_context_.slavelist[slave].Obytes < kCspPdoSize) {
      RCLCPP_ERROR(kLogger, "Invalid RxPDO for joint '%s'", joint_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    mode = kCyclicSynchronousPositionMode;
    std::memcpy(outputs, &control_word, sizeof(control_word));
    std::memcpy(outputs + sizeof(control_word), &target_position, sizeof(target_position));
    std::memcpy(
      outputs + sizeof(control_word) + sizeof(target_position), &mode, sizeof(mode));
  }

  ecx_send_processdata(&ethercat_context_);
  const int work_counter = ecx_receive_processdata(&ethercat_context_, EC_TIMEOUTRET);
  if (work_counter < expected_work_counter_) {
    RCLCPP_ERROR(
      kLogger, "Failed to start CSP process data: WKC %d, expected %d",
      work_counter, expected_work_counter_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  csp_process_data_ready_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (!ethercat_initialized_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Best-effort shutdown: attempt every slave even if one SDO write fails.
  for (const auto slave : slave_ids_) {
    uint16_t control_word = kControlDisableVoltage;
    if (ecx_SDOwrite(
        &ethercat_context_, slave, kControlWord, 0x00, FALSE, sizeof(control_word),
        &control_word, EC_TIMEOUTRXM) <= 0)
    {
      RCLCPP_WARN(kLogger, "Failed to disable voltage for slave %u", slave);
    }
  }

  csp_process_data_ready_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  // ecx_close releases the raw socket and all SOEM context resources.
  if (ethercat_initialized_) {
    ecx_close(&ethercat_context_);
  }

  ethercat_initialized_ = false;
  ethercat_operational_ = false;
  csp_process_data_ready_ = false;
  io_map_.clear();
  expected_work_counter_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RasclHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Interfaces hold pointers into vectors allocated in on_init(); those vectors
  // must not be resized while the controller manager owns the interfaces.
  for (auto i = 0u; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, "actual_position", &hw_actual_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, "status_word", &hw_status_words_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RasclHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type RasclHardwareInterface::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & period)
{
  if (!ethercat_initialized_ || !ethercat_operational_ || !csp_process_data_ready_) {
    return hardware_interface::return_type::ERROR;
  }

  ecx_send_processdata(&ethercat_context_);
  const int work_counter = ecx_receive_processdata(&ethercat_context_, EC_TIMEOUTRET);
  if (work_counter < expected_work_counter_) {
    RCLCPP_WARN(
      kLogger, "Incomplete EtherCAT process data: WKC %d, expected %d; using SDO state fallback",
      work_counter, expected_work_counter_);

    for (auto i = 0u; i < hw_positions_.size(); ++i) {
      const auto slave = slave_ids_[i];
      const auto & joint_name = info_.joints[i].name;
      uint16_t status_word = 0;
      int32_t actual_position = 0;
      int size = sizeof(status_word);

      if (ecx_SDOread(
          &ethercat_context_, slave, kStatusWord, 0x00, FALSE, &size, &status_word,
          EC_TIMEOUTRXM) <= 0)
      {
        RCLCPP_ERROR(kLogger, "Failed SDO fallback statusword read for joint '%s'",
          joint_name.c_str());
        return hardware_interface::return_type::ERROR;
      }

      size = sizeof(actual_position);
      if (ecx_SDOread(
          &ethercat_context_, slave, kPositionActualValue, 0x00, FALSE, &size,
          &actual_position, EC_TIMEOUTRXM) <= 0)
      {
        RCLCPP_ERROR(kLogger, "Failed SDO fallback actual-position read for joint '%s'",
          joint_name.c_str());
        return hardware_interface::return_type::ERROR;
      }

      const double previous_position = hw_positions_[i];
      hw_actual_positions_[i] = static_cast<double>(actual_position);
      hw_positions_[i] = static_cast<double>(actual_position) / drive_units_per_radian_[i];

      const double period_seconds = period.seconds();
      hw_velocities_[i] = period_seconds > 0.0 ?
        (hw_positions_[i] - previous_position) / period_seconds : 0.0;
      hw_status_words_[i] = static_cast<double>(status_word);
      last_status_words_[i] = status_word;
    }

    return hardware_interface::return_type::OK;
  }

  for (auto i = 0u; i < hw_positions_.size(); ++i) {
    const auto slave = slave_ids_[i];
    uint16_t status_word = 0;
    int32_t actual_position = 0;
    int8_t mode_display = 0;
    const uint8_t * inputs = ethercat_context_.slavelist[slave].inputs;
    if (inputs == nullptr || ethercat_context_.slavelist[slave].Ibytes < kCspPdoSize) {
      RCLCPP_ERROR(kLogger, "Invalid TxPDO for joint '%s'", info_.joints[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    std::memcpy(&status_word, inputs, sizeof(status_word));
    std::memcpy(&actual_position, inputs + sizeof(status_word), sizeof(actual_position));
    std::memcpy(
      &mode_display, inputs + sizeof(status_word) + sizeof(actual_position),
      sizeof(mode_display));
    if (mode_display != kCyclicSynchronousPositionMode) {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' left CSP mode: 0x6061 = %d",
        info_.joints[i].name.c_str(), mode_display);
      return hardware_interface::return_type::ERROR;
    }

    // Convert counts to radians and estimate velocity by finite difference.
    const double previous_position = hw_positions_[i];
    hw_actual_positions_[i] = static_cast<double>(actual_position);
    hw_positions_[i] = static_cast<double>(actual_position) / drive_units_per_radian_[i];

    const double period_seconds = period.seconds();
    hw_velocities_[i] = period_seconds > 0.0 ?
      (hw_positions_[i] - previous_position) / period_seconds : 0.0;
    hw_status_words_[i] = static_cast<double>(status_word);
    last_status_words_[i] = status_word;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RasclHardwareInterface::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (!ethercat_initialized_ || !ethercat_operational_ || !csp_process_data_ready_) {
    return hardware_interface::return_type::ERROR;
  }

  for (auto i = 0u; i < hw_commands_.size(); ++i) {
    const double requested_position = hw_commands_[i];
    if (!std::isfinite(requested_position)) {
      RCLCPP_ERROR(
        kLogger, "Refusing non-finite position command for joint '%s'",
        info_.joints[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }

    // Enforce the final safety boundary before converting radians to counts.
    const double limited_position = std::max(
      lower_position_limits_[i], std::min(requested_position, upper_position_limits_[i]));
    if (limited_position != requested_position) {
      if (!command_limit_warning_active_[i]) {
        RCLCPP_WARN(
          kLogger, "Clamping joint '%s' command %.6f rad to %.6f rad (limits: %.6f..%.6f)",
          info_.joints[i].name.c_str(), requested_position, limited_position,
          lower_position_limits_[i], upper_position_limits_[i]);
      }
      command_limit_warning_active_[i] = true;
    } else {
      command_limit_warning_active_[i] = false;
    }

    const double target_drive_units = limited_position * drive_units_per_radian_[i];
    if (!std::isfinite(target_drive_units) ||
      target_drive_units < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      target_drive_units > static_cast<double>(std::numeric_limits<int32_t>::max()))
    {
      RCLCPP_ERROR(
        kLogger, "Position command for joint '%s' is outside the drive-unit range",
        info_.joints[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    const int32_t target_position = static_cast<int32_t>(std::llround(target_drive_units));

    if ((last_status_words_[i] & kStatusFault) != 0) {
      RCLCPP_ERROR(
        kLogger, "Refusing command for joint '%s' because last statusword is fault: 0x%04X",
        info_.joints[i].name.c_str(), last_status_words_[i]);
      return hardware_interface::return_type::ERROR;
    }

    const auto slave = slave_ids_[i];
    const auto & joint_name = info_.joints[i].name;
    uint8_t * outputs = ethercat_context_.slavelist[slave].outputs;
    if (outputs == nullptr || ethercat_context_.slavelist[slave].Obytes < kCspPdoSize) {
      RCLCPP_ERROR(kLogger, "Invalid RxPDO for joint '%s'", joint_name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    const uint16_t control_word = kControlEnableOperation;
    const int8_t mode = kCyclicSynchronousPositionMode;
    std::memcpy(outputs, &control_word, sizeof(control_word));
    std::memcpy(outputs + sizeof(control_word), &target_position, sizeof(target_position));
    std::memcpy(
      outputs + sizeof(control_word) + sizeof(target_position), &mode, sizeof(mode));
  }

  ecx_send_processdata(&ethercat_context_);

  return hardware_interface::return_type::OK;
}

}  // namespace rascl_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  rascl_hardware_interface::RasclHardwareInterface,
  hardware_interface::SystemInterface)
