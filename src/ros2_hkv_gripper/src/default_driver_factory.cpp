// Copyright (c) 2024
// Licensed under BSD-3-Clause License

#include "hkv_gripper_controller/default_driver_factory.hpp"
#include "hkv_gripper_controller/default_driver.hpp"
#include "hkv_gripper_controller/default_serial.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hkv_gripper_controller
{

const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

constexpr auto kSerialPortParamName = "serial_port";
constexpr auto kSerialPortParamDefault = "/dev/ttyUSB0";

constexpr auto kBaudrateParamName = "baud_rate";
constexpr auto kBaudrateParamDefault = 1000000;

constexpr auto kTimeoutParamName = "timeout";
constexpr auto kTimeoutParamDefault = 1000;  // milliseconds

constexpr auto kSlaveAddressParamName = "slave_address";
constexpr auto kSlaveAddressParamDefault = 0x01;

std::unique_ptr<Driver> DefaultDriverFactory::create(
    const hardware_interface::HardwareInfo& info) const
{
  // Parse parameters from hardware info
  std::string serial_port = kSerialPortParamDefault;
  int baud_rate = kBaudrateParamDefault;
  int timeout = kTimeoutParamDefault;
  uint8_t slave_address = kSlaveAddressParamDefault;

  // Read from hardware parameters
  auto it = info.hardware_parameters.find(kSerialPortParamName);
  if (it != info.hardware_parameters.end())
  {
    serial_port = it->second;
  }

  it = info.hardware_parameters.find(kBaudrateParamName);
  if (it != info.hardware_parameters.end())
  {
    baud_rate = std::stoi(it->second);
  }

  it = info.hardware_parameters.find(kTimeoutParamName);
  if (it != info.hardware_parameters.end())
  {
    timeout = std::stoi(it->second);
  }

  it = info.hardware_parameters.find(kSlaveAddressParamName);
  if (it != info.hardware_parameters.end())
  {
    slave_address = std::stoi(it->second);
  }

  RCLCPP_INFO(kLogger, "Creating driver with serial_port=%s, baud_rate=%d, timeout=%d, slave_address=%d",
              serial_port.c_str(), baud_rate, timeout, slave_address);

  // Create serial interface with parameters
  auto serial = std::make_unique<DefaultSerial>(serial_port, baud_rate, timeout / 1000.0);

  // Create driver with serial interface
  auto driver = std::make_unique<DefaultDriver>(std::move(serial));
  driver->setSlaveAddress(slave_address);
  driver->setPositionRange(100, 0);
  driver->setDefaults(1000, 50);

  return driver;
}

std::unique_ptr<Driver> DefaultDriverFactory::create_driver(
    const hardware_interface::HardwareInfo& /*info*/) const
{
  return std::make_unique<DefaultDriver>();
}

}  // namespace hkv_gripper_controller
