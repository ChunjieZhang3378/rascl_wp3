#ifndef RASCL_HARDWARE_INTERFACE__RASCL_HARDWARE_INTERFACE_HPP_
#define RASCL_HARDWARE_INTERFACE__RASCL_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "soem/soem.h"

namespace rascl_hardware_interface
{

/**
 * @brief ros2_control system interface for the RASCL EtherCAT drives.
 *
 * The interface uses SOEM to configure the drives, home each joint, and operate
 * them in CiA 402 Cyclic Synchronous Position mode. Joint configuration,
 * conversion factors, and optional position limits are read from the
 * robot's ros2_control URDF section.
 */
class RasclHardwareInterface : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Validate the URDF hardware configuration and allocate joint storage.
   * @param info Parsed ros2_control hardware configuration.
   * @return SUCCESS for valid configuration; otherwise ERROR.
   *
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Open the EtherCAT adapter and discover and configure the slaves.
   * @param previous_state Lifecycle state before configuration.
   * @return SUCCESS when communication is ready; otherwise ERROR.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Enable and home all drives, then select CSP mode.
   * @param previous_state Lifecycle state before activation.
   * @return SUCCESS when every drive is homed and enabled; otherwise ERROR.
   * @warning This method can move the physical robot.
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Disable drive voltage while retaining the EtherCAT connection.
   * @param previous_state Lifecycle state before deactivation.
   * @return SUCCESS after attempting to disable all configured drives.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Close the EtherCAT connection and reset communication state.
   * @param previous_state Lifecycle state before cleanup.
   * @return SUCCESS.
   */
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Export position, velocity, raw-position, and status-word states. */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /** @brief Export one position command interface for every configured joint. */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Receive status words and actual positions from all drives using TxPDOs.
   * @param time Current controller time; currently unused.
   * @param period Time since the previous cycle, used to estimate velocity.
   * @return OK after all reads succeed; otherwise ERROR.
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Validate, limit, convert, and send position targets using RxPDOs.
   * @param time Current controller time; currently unused.
   * @param period Time since the previous cycle; currently unused.
   * @return OK after all required writes succeed; otherwise ERROR.
   *
   * Commands outside configured URDF limits are clamped.
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // EtherCAT connection and lifecycle state.
  bool ethercat_initialized_{false};
  bool ethercat_operational_{false};
  bool csp_process_data_ready_{false};
  ecx_contextt ethercat_context_{};
  std::string adapter_;
  std::vector<uint8_t> io_map_;
  int expected_work_counter_{0};

  // ros2_control state and command values, indexed in URDF joint order.
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_actual_positions_;
  std::vector<double> hw_status_words_;
  std::vector<double> hw_commands_;

  // Software position limits.
  std::vector<double> lower_position_limits_;
  std::vector<double> upper_position_limits_;
  std::vector<bool> command_limit_warning_active_;

  // Per-joint EtherCAT addressing and drive configuration.
  std::vector<uint16_t> slave_ids_;
  std::vector<double> drive_units_per_radian_;
  std::vector<uint16_t> last_status_words_;
};

}  // namespace rascl_hardware_interface

#endif  // RASCL_HARDWARE_INTERFACE__RASCL_HARDWARE_INTERFACE_HPP_
