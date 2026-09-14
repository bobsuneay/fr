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

#include <cstddef>

#include "hkv_gripper_controller/modbus_utils.hpp"

namespace hkv_gripper_controller
{
namespace modbus_utils
{

uint16_t calculateCRC16(const std::vector<uint8_t>& data)
{
  uint16_t crc = 0xFFFF;
  
  for (uint8_t byte : data) {
    crc ^= byte;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

std::vector<uint8_t> buildReadRegistersFrame(uint8_t slave_id, uint16_t start_addr, uint16_t count)
{
  std::vector<uint8_t> frame;
  frame.reserve(8);
  
  frame.push_back(slave_id);
  frame.push_back(0x03);  // Function code: Read Holding Registers
  frame.push_back((start_addr >> 8) & 0xFF);
  frame.push_back(start_addr & 0xFF);
  frame.push_back((count >> 8) & 0xFF);
  frame.push_back(count & 0xFF);
  
  uint16_t crc = calculateCRC16(frame);
  frame.push_back(crc & 0xFF);       // CRC low byte
  frame.push_back((crc >> 8) & 0xFF);  // CRC high byte
  
  return frame;
}

std::vector<uint8_t> buildReadInputRegistersFrame(uint8_t slave_id, uint16_t start_addr, uint16_t count)
{
  std::vector<uint8_t> frame;
  frame.reserve(8);

  frame.push_back(slave_id);
  frame.push_back(0x04);  // Function code: Read Input Registers
  frame.push_back((start_addr >> 8) & 0xFF);
  frame.push_back(start_addr & 0xFF);
  frame.push_back((count >> 8) & 0xFF);
  frame.push_back(count & 0xFF);

  uint16_t crc = calculateCRC16(frame);
  frame.push_back(crc & 0xFF);       // CRC low byte
  frame.push_back((crc >> 8) & 0xFF);  // CRC high byte

  return frame;
}

std::vector<uint8_t> buildWriteSingleRegisterFrame(uint8_t slave_id, uint16_t register_addr, uint16_t value)
{
  std::vector<uint8_t> frame;
  frame.reserve(8);
  
  frame.push_back(slave_id);
  frame.push_back(0x06);  // Function code: Write Single Register
  frame.push_back((register_addr >> 8) & 0xFF);
  frame.push_back(register_addr & 0xFF);
  frame.push_back((value >> 8) & 0xFF);
  frame.push_back(value & 0xFF);
  
  uint16_t crc = calculateCRC16(frame);
  frame.push_back(crc & 0xFF);       // CRC low byte
  frame.push_back((crc >> 8) & 0xFF);  // CRC high byte
  
  return frame;
}

std::vector<uint8_t> buildWriteMultipleRegistersFrame(uint8_t slave_id, uint16_t start_addr, const std::vector<uint16_t>& values)
{
  std::vector<uint8_t> frame;
  frame.reserve(9 + values.size() * 2);

  frame.push_back(slave_id);
  frame.push_back(0x10);  // Function code: Write Multiple Registers
  frame.push_back((start_addr >> 8) & 0xFF);
  frame.push_back(start_addr & 0xFF);

  uint16_t count = static_cast<uint16_t>(values.size());
  frame.push_back((count >> 8) & 0xFF);
  frame.push_back(count & 0xFF);

  uint8_t byte_count = static_cast<uint8_t>(count * 2);
  frame.push_back(byte_count);

  for (uint16_t value : values) {
    frame.push_back((value >> 8) & 0xFF);
    frame.push_back(value & 0xFF);
  }

  uint16_t crc = calculateCRC16(frame);
  frame.push_back(crc & 0xFF);       // CRC low byte
  frame.push_back((crc >> 8) & 0xFF);  // CRC high byte

  return frame;
}

bool verifyCRC(const std::vector<uint8_t>& response)
{
  if (response.size() < 3) {
    return false;
  }
  
  std::vector<uint8_t> data(response.begin(), response.end() - 2);
  uint16_t calculated_crc = calculateCRC16(data);
  uint16_t received_crc = response[response.size() - 2] | (response[response.size() - 1] << 8);
  
  return calculated_crc == received_crc;
}

std::vector<int16_t> parseRegisterValues(const std::vector<uint8_t>& response)
{
  std::vector<int16_t> values;
  
  if (response.size() < 5) {
    return values;
  }

  // Accept both holding (0x03) and input (0x04) register responses
  if (response[1] != 0x03 && response[1] != 0x04) {
    return values;
  }
  
  uint8_t byte_count = response[2];
  std::size_t expected_size = 5 + static_cast<std::size_t>(byte_count);
  
  if (response.size() < expected_size) {
    return values;
  }
  
  for (std::size_t i = 3; i < 3 + static_cast<std::size_t>(byte_count); i += 2) {
    int16_t value = static_cast<int16_t>((response[i] << 8) | response[i + 1]);
    values.push_back(value);
  }
  
  return values;
}

}  // namespace modbus_utils
}  // namespace hkv_gripper_controller
