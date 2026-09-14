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

#include <cstdint>
#include <vector>

namespace hkv_gripper_controller
{

/**
 * @brief Utility functions for Modbus RTU protocol
 */
namespace modbus_utils
{

/**
 * @brief Calculate Modbus RTU CRC16 (little-endian)
 * @param data Data to calculate CRC for
 * @return CRC16 value
 */
uint16_t calculateCRC16(const std::vector<uint8_t>& data);

/**
 * @brief Build a Modbus RTU read holding registers frame (function code 0x03)
 * @param slave_id Slave device ID
 * @param start_addr Starting register address
 * @param count Number of registers to read
 * @return Complete Modbus RTU frame with CRC
 */
std::vector<uint8_t> buildReadRegistersFrame(uint8_t slave_id, uint16_t start_addr, uint16_t count);

/**
 * @brief Build a Modbus RTU read input registers frame (function code 0x04)
 * @param slave_id Slave device ID
 * @param start_addr Starting register address
 * @param count Number of registers to read
 * @return Complete Modbus RTU frame with CRC
 */
std::vector<uint8_t> buildReadInputRegistersFrame(uint8_t slave_id, uint16_t start_addr, uint16_t count);

/**
 * @brief Build a Modbus RTU write single register frame (function code 0x06)
 * @param slave_id Slave device ID
 * @param register_addr Register address
 * @param value Value to write
 * @return Complete Modbus RTU frame with CRC
 */
std::vector<uint8_t> buildWriteSingleRegisterFrame(uint8_t slave_id, uint16_t register_addr, uint16_t value);

/**
 * @brief Build a Modbus RTU write multiple registers frame (function code 0x10)
 * @param slave_id Slave device ID
 * @param start_addr Starting register address
 * @param values Register values to write
 * @return Complete Modbus RTU frame with CRC
 */
std::vector<uint8_t> buildWriteMultipleRegistersFrame(uint8_t slave_id, uint16_t start_addr, const std::vector<uint16_t>& values);

/**
 * @brief Verify CRC of a Modbus RTU response
 * @param response Complete response frame including CRC
 * @return True if CRC is valid
 */
bool verifyCRC(const std::vector<uint8_t>& response);

/**
 * @brief Parse register values from read response (function code 0x03 or 0x04)
 * @param response Response frame from read registers command
 * @return Vector of register values (16-bit signed)
 */
std::vector<int16_t> parseRegisterValues(const std::vector<uint8_t>& response);

}  // namespace modbus_utils

}  // namespace hkv_gripper_controller
