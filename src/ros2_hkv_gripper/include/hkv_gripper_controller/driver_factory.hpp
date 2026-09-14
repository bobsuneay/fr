// Copyright (c) 2024
// Licensed under BSD-3-Clause License

#pragma once

#include <memory>
#include <hardware_interface/hardware_info.hpp>
#include "hkv_gripper_controller/driver.hpp"

namespace hkv_gripper_controller
{

/**
 * Factory interface for creating drivers.
 * Allows for easy testing and mocking.
 */
class DriverFactory
{
public:
  virtual ~DriverFactory() = default;
  
  /**
   * Create a driver from hardware info.
   * @param info Hardware information from URDF
   * @return Unique pointer to driver
   */
  virtual std::unique_ptr<Driver> create(const hardware_interface::HardwareInfo& info) const = 0;
};

}  // namespace hkv_gripper_controller
