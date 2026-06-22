#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rascl_hardware_interface/rascl_hardware_interface.hpp"

namespace
{

hardware_interface::HardwareInfo load_hardware_info()
{
  std::ifstream file(RASCL_TEST_URDF_PATH);
  if (!file.is_open()) {
    throw std::runtime_error(
            std::string("Could not open test URDF: ") + RASCL_TEST_URDF_PATH);
  }

  const std::string urdf(
    (std::istreambuf_iterator<char>(file)),
    std::istreambuf_iterator<char>());
  const auto hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf);

  if (hardware_infos.empty()) {
    throw std::runtime_error("The URDF contains no ros2_control hardware configuration");
  }
  return hardware_infos.front();
}

TEST(RasclHardwareInterface, InitializesFromProjectUrdf)
{
  auto info = load_hardware_info();
  rascl_hardware_interface::RasclHardwareInterface hardware;

  ASSERT_EQ(
    hardware.on_init(info),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto state_interfaces = hardware.export_state_interfaces();
  const auto command_interfaces = hardware.export_command_interfaces();
  EXPECT_EQ(state_interfaces.size(), info.joints.size() * 4u);
  EXPECT_EQ(command_interfaces.size(), info.joints.size());
}

TEST(RasclHardwareInterface, RejectsMissingAdapter)
{
  auto info = load_hardware_info();
  info.hardware_parameters.erase("adapter");
  rascl_hardware_interface::RasclHardwareInterface hardware;

  EXPECT_EQ(
    hardware.on_init(info),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(RasclHardwareInterface, RejectsMissingPositionScale)
{
  auto info = load_hardware_info();
  ASSERT_FALSE(info.joints.empty());
  info.joints.front().parameters.erase("drive_units_per_radian");
  rascl_hardware_interface::RasclHardwareInterface hardware;

  EXPECT_EQ(
    hardware.on_init(info),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(RasclHardwareInterface, RejectsMissingPositionCommandInterface)
{
  auto info = load_hardware_info();
  ASSERT_FALSE(info.joints.empty());
  info.joints.front().command_interfaces.clear();
  rascl_hardware_interface::RasclHardwareInterface hardware;

  EXPECT_EQ(
    hardware.on_init(info),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(RasclHardwareInterface, AcceptsJointWithoutPositionLimits)
{
  auto info = load_hardware_info();
  ASSERT_FALSE(info.joints.empty());
  ASSERT_FALSE(info.joints.back().command_interfaces.empty());
  info.joints.back().command_interfaces.front().min.clear();
  info.joints.back().command_interfaces.front().max.clear();
  rascl_hardware_interface::RasclHardwareInterface hardware;

  EXPECT_EQ(
    hardware.on_init(info),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST(RasclHardwareInterface, RejectsMissingVelocityStateInterface)
{
  auto info = load_hardware_info();
  ASSERT_FALSE(info.joints.empty());
  auto & state_interfaces = info.joints.front().state_interfaces;
  for (
    auto interface = state_interfaces.begin(); interface != state_interfaces.end(); ++interface)
  {
    if (interface->name == "velocity") {
      state_interfaces.erase(interface);
      break;
    }
  }
  rascl_hardware_interface::RasclHardwareInterface hardware;

  EXPECT_EQ(
    hardware.on_init(info),
    hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
