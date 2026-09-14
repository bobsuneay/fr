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

#include "hkv_gripper_controller/serial.hpp"
#include <memory>
#include <vector>
#include <cstdint>

namespace hkv_gripper_controller
{

/**
 * @brief Finger state structure
 */
struct FingerState
{
  bool is_valid = false;
  
  // 0x0000-0x0005: 力传感器数据
  int16_t x1_value = 0;          // X1数值 (0x0000)
  int16_t y1_value = 0;          // Y1数值 (0x0001)
  int16_t z1_value = 0;          // Z1数值 (0x0002)
  int16_t x2_value = 0;          // X2数值 (0x0003)
  int16_t y2_value = 0;          // Y2数值 (0x0004)
  int16_t z2_value = 0;          // Z2数值 (0x0005)
  
  // 0x0006-0x0009: 状态与控制寄存器
  uint16_t status_register = 0;    // 状态寄存器(0x0006)
  uint16_t softness_register = 0;  // 软硬度寄存器(0x0007)
  uint16_t position_register = 0;  // 原始位置寄存器值(0x0008)
  uint16_t current_register = 0;   // 电流寄存器(0x0009)
  
  // 计算字段
  uint8_t position = 0;            // Scaled position (0-255, derived from寄存器0x0008)
  bool is_moving = false;          // Derived from status register
};

/**
 * @brief Driver interface for gripper communication
 */
class Driver
{
public:
  virtual ~Driver() = default;

  /**
   * @brief Connect to the gripper
   * @return True if connection successful
   */
  virtual bool connect() = 0;

  /**
   * @brief Disconnect from the gripper
   */
  virtual void disconnect() = 0;

  /**
   * @brief Check if connected
   * @return True if connected
   */
  virtual bool isConnected() const = 0;

  /**
   * @brief Activate gripper (initialize and calibrate)
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivate gripper
   */
  virtual void deactivate() = 0;

  /**
   * @brief Set slave address for Modbus communication
   * @param slave_id Modbus slave ID
   */
  virtual void setSlaveAddress(uint8_t slave_id) = 0;

  /**
   * @brief Configure motion ranges (位置寄存器范围)
   * @param open_register   Register value when fully open (e.g., 100)
   * @param closed_register Register value when fully closed (e.g., 0)
   */
  virtual void setPositionRange(uint16_t open_register, uint16_t closed_register) = 0;

  /**
   * @brief Configure默认速度/目标力（寄存器值）
   * @param speed_register Motion speed register value (200~1500)
   * @param force_percent  Target force percent (1~100)
   */
  virtual void setDefaults(uint16_t speed_register, uint16_t force_percent) = 0;

  /**
   * @brief Set gripper position with speed and force
   * @param position Target position (0-255, 0=open, 255=closed)
   * @param speed Speed (0-255)
   * @param force Force (0-255)
   */
  virtual void grip(uint8_t position, uint8_t speed, uint8_t force) = 0;

  /**
   * @brief Read finger state from gripper
   * @return Finger state structure
   */
  virtual FingerState readFingerState() = 0;

  /**
   * @brief Send grip command
   * @param speed Optional speed parameter (200-1500). 0 means use default.
   * @return True if command sent successfully
   */
  virtual bool grip(uint16_t speed = 0) = 0;

  /**
   * @brief Send release command
   * @param speed Optional speed parameter (200-1500). 0 means use default.
   * @return True if command sent successfully
   */
  virtual bool release(uint16_t speed = 0) = 0;

  /**
   * @brief Read finger state registers
   * @param start_addr Starting register address
   * @param count Number of registers to read
   * @return Register values, empty if failed
   */
  virtual std::vector<int16_t> readFingerState(uint16_t start_addr, uint16_t count) = 0;
};

}  // namespace hkv_gripper_controller
