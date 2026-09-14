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
#include <string>
#include <memory>

namespace hkv_gripper_controller
{

/**
 * @brief Default implementation of Serial interface using Linux/Unix APIs
 */
class DefaultSerial : public Serial
{
public:
  /**
   * @brief Constructor
   * @param port Serial port name (e.g., "/dev/ttyUSB0")
   * @param baudrate Baud rate
   * @param timeout Timeout in seconds
   */
  DefaultSerial(const std::string& port, uint32_t baudrate, double timeout = 1.0);

  ~DefaultSerial() override;

  bool open() override;
  void close() override;
  bool isOpen() const override;
  size_t write(const std::vector<uint8_t>& data) override;
  std::vector<uint8_t> read(size_t size) override;
  size_t available() const override;
  void flush() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hkv_gripper_controller
