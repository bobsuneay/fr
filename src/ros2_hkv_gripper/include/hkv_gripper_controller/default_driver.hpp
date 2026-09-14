// Copyright 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "hkv_gripper_controller/driver.hpp"
#include "hkv_gripper_controller/serial.hpp"
#include <memory>
#include <chrono>
#include <mutex>

namespace hkv_gripper_controller
{

/**
 * @brief Default implementation of Driver using Modbus RTU
 */
class DefaultDriver : public Driver
{
public:
  /**
   * @brief Default constructor (for factory pattern)
   */
  DefaultDriver();

  /**
   * @brief Constructor with serial interface
   * @param serial Serial communication interface
   */
  explicit DefaultDriver(std::shared_ptr<Serial> serial);

  ~DefaultDriver() override = default;

  // Setter methods for factory pattern
  void setSerial(std::unique_ptr<Serial> serial);
  void setSlaveAddress(uint8_t slave_id);
  void setPositionRange(uint16_t open_register, uint16_t closed_register) override;
  void setDefaults(uint16_t speed_register, uint16_t force_percent) override;

  bool connect() override;
  void disconnect() override;
  bool isConnected() const override;
  
  void activate() override;
  void deactivate() override;
  void grip(uint8_t position, uint8_t speed, uint8_t force) override;
  FingerState readFingerState() override;
  
  // Legacy interface
  bool grip(uint16_t speed = 0) override;
  bool release(uint16_t speed = 0) override;
  std::vector<int16_t> readFingerState(uint16_t start_addr, uint16_t count) override;

private:
  uint16_t mapPositionByteToRegister(uint8_t position_byte) const;
  uint16_t mapSpeedByteToRegister(uint8_t speed_byte) const;
  uint16_t mapForceByteToPercent(uint8_t force_byte) const;

  bool writeSingleWithReadback(uint16_t register_addr, uint16_t value);
  bool writeMultipleWithReadback(uint16_t start_addr, const std::vector<uint16_t>& values);

  /**
   * @brief Write frame to serial port and read response
   * @param frame Frame to write
   * @param expected_response_size Expected response size
   * @return Response data, empty if failed
   */
  std::vector<uint8_t> writeAndRead(const std::vector<uint8_t>& frame, size_t expected_response_size);

  std::shared_ptr<Serial> serial_;
  std::recursive_mutex serial_mutex_;
  uint8_t slave_id_;
  bool connected_{false};
  bool position_mode_set_{false};
  uint16_t open_register_{100};
  uint16_t closed_register_{0};
  uint16_t default_speed_register_{1000};
  uint16_t default_force_percent_{50};
  
  static constexpr auto kResponseDelay = std::chrono::milliseconds(2);
  static constexpr auto kMaxWaitTime = std::chrono::milliseconds(20);
};

}  // namespace hkv_gripper_controller
