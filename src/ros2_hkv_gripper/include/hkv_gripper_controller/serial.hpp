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
#include <string>
#include <vector>

namespace hkv_gripper_controller
{

/**
 * @brief Interface for serial communication
 */
class Serial
{
public:
  virtual ~Serial() = default;

  /**
   * @brief Open serial port
   */
  virtual bool open() = 0;

  /**
   * @brief Close serial port
   */
  virtual void close() = 0;

  /**
   * @brief Check if port is open
   */
  virtual bool isOpen() const = 0;

  /**
   * @brief Write data to serial port
   * @param data Data to write
   * @return Number of bytes written
   */
  virtual size_t write(const std::vector<uint8_t>& data) = 0;

  /**
   * @brief Read data from serial port
   * @param size Number of bytes to read
   * @return Data read from port
   */
  virtual std::vector<uint8_t> read(size_t size) = 0;

  /**
   * @brief Get number of bytes available to read
   * @return Number of bytes available
   */
  virtual size_t available() const = 0;

  /**
   * @brief Flush the serial port buffers
   */
  virtual void flush() = 0;
};

}  // namespace hkv_gripper_controller
