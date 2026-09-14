// Copyright (c) 2024
// Licensed under BSD-3-Clause License

#pragma once

#include <memory>
#include <hardware_interface/hardware_info.hpp>
#include "hkv_gripper_controller/driver_factory.hpp"
#include "hkv_gripper_controller/driver.hpp"

namespace hkv_gripper_controller
{

/**
 * Default factory implementation that creates DefaultDriver instances.
 */
class DefaultDriverFactory : public DriverFactory
{
public:
  DefaultDriverFactory() = default;

  /**
   * Create a default driver from hardware info.
   * @param info Hardware information from URDF
   * @return Unique pointer to driver
   */
  std::unique_ptr<Driver> create(const hardware_interface::HardwareInfo& info) const override;

protected:
  // Seam for testing
  virtual std::unique_ptr<Driver> create_driver(const hardware_interface::HardwareInfo& info) const;
};

}  // namespace hkv_gripper_controller
