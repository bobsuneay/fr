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

#include "hkv_gripper_controller/default_driver.hpp"
#include "hkv_gripper_controller/modbus_utils.hpp"
#include <thread>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace hkv_gripper_controller
{

// Register addresses (Modbus RTU, see EG-9801 protocol)
constexpr uint16_t kX1Register = 0x0000;
constexpr uint16_t kStatusRegister = 0x0006;
constexpr uint16_t kSoftnessRegister = 0x0007;
constexpr uint16_t kPositionRegister = 0x0008;
constexpr uint16_t kCurrentRegister = 0x0009;
constexpr uint16_t kSpeedRegister = 0x000A;
constexpr uint16_t kCommandRegister = 0x000B;
constexpr uint16_t kProtocolRegister = 0x000C;
constexpr uint16_t kDeviceIdRegister = 0x000D;
constexpr uint16_t kControlModeRegister = 0x000E;
constexpr uint16_t kPositionModeSpeedRegister = 0x000F;
constexpr uint16_t kPositionModeTargetRegister = 0x0010;
constexpr uint16_t kForceCalibrationRegister = 0x0011;
constexpr uint16_t kTargetForceRegister = 0x0012;
constexpr uint16_t kFactoryInfoStartRegister = 0x0013;

// Control mode values
constexpr uint16_t kControlModeAuto = 0x0000;
constexpr uint16_t kControlModePosition = 0x0001;

// Protocol values
constexpr uint16_t kProtocolModbus = 0x0002;

DefaultDriver::DefaultDriver()
  : slave_id_(0x01), connected_(false)
{
}

DefaultDriver::DefaultDriver(std::shared_ptr<Serial> serial)
  : serial_(serial), slave_id_(0x01), connected_(false)
{
}

void DefaultDriver::setSerial(std::unique_ptr<Serial> serial)
{
  serial_ = std::move(serial);
}

void DefaultDriver::setSlaveAddress(uint8_t slave_id)
{
  slave_id_ = slave_id;
}

void DefaultDriver::setPositionRange(uint16_t open_register, uint16_t closed_register)
{
  // 保证范围合法
  open_register_ = std::max(open_register, closed_register);
  closed_register_ = std::min(open_register, closed_register);
}

void DefaultDriver::setDefaults(uint16_t speed_register, uint16_t force_percent)
{
  // 速度寄存器范围 200~1500
  default_speed_register_ = static_cast<uint16_t>(std::clamp<int>(speed_register, 200, 1500));
  // 力度 1~100
  default_force_percent_ = static_cast<uint16_t>(std::clamp<int>(force_percent, 1, 100));
}

bool DefaultDriver::connect()
{
  if (serial_ && serial_->open()) {
    connected_ = true;
    return true;
  }
  return false;
}

void DefaultDriver::disconnect()
{
  if (serial_) {
    serial_->close();
  }
  connected_ = false;
}

bool DefaultDriver::isConnected() const
{
  return connected_ && serial_ && serial_->isOpen();
}

void DefaultDriver::activate()
{
  if (!isConnected()) {
    throw std::runtime_error("Cannot activate: not connected");
  }

  // 初始化阶段设置为自动模式，避免设备因历史目标值产生动作
  position_mode_set_ = false;
  writeSingleWithReadback(kControlModeRegister, kControlModeAuto);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void DefaultDriver::deactivate()
{
  if (!isConnected()) {
    return;
  }

  position_mode_set_ = false;

  // 释放命令，速度保持默认
  std::vector<uint16_t> values = {default_speed_register_, 0x0000};
  writeMultipleWithReadback(kSpeedRegister, values);
}

void DefaultDriver::grip(uint8_t position, uint8_t speed, uint8_t force)
{
  if (!isConnected()) {
    throw std::runtime_error("Cannot grip: not connected");
  }

  const uint16_t force_reg = static_cast<uint16_t>(std::lround(1.0+99.0*force/255.0));

  // 寄存器映射：速度(0x000F)、位置(0x0010)，目标力寄存器暂时不写
  const uint16_t speed_reg = mapSpeedByteToRegister(speed);
  const uint16_t position_reg = mapPositionByteToRegister(position);

  // 首次下发时切换位置模式，并在帧间回读校验（仅一次）
  if (!position_mode_set_) {
    if (!writeSingleWithReadback(kTargetForceRegister, force_reg)) {
      throw std::runtime_error("Failed to set target force register");
    }
    if (!writeSingleWithReadback(kControlModeRegister, kControlModePosition)) {
      throw std::runtime_error("Failed to set control mode with verification");
    }
    position_mode_set_ = true;
  }

  // 写入速度+位置（多寄存器写）
  // 位置模式下，目标位置寄存器在硬件插补运动时会持续改变内部实际状态，
  // 导致严格的回读对比（期望读到的值与写入的指令值完全一致）很容易失败。
  // 因此，这里只下发指令而不强制要求回读匹配，失败时不抛出异常。
  std::vector<uint16_t> values = {speed_reg, position_reg};
  if (!writeMultipleWithReadback(kPositionModeSpeedRegister, values)) {
    throw std::runtime_error("Position command was not verified");
  }
}

FingerState DefaultDriver::readFingerState()
{
  if (!isConnected()) {
    throw std::runtime_error("Cannot read state: not connected");
  }

  // 读取 0x0000 开始的 10 个保持寄存器（协议表定义的设备信息）
  auto frame = modbus_utils::buildReadRegistersFrame(slave_id_, kX1Register, 10);

  size_t expected_size = 5 + 2 * 10;  // 10 registers
  auto response = writeAndRead(frame, expected_size);

  FingerState state{};
  state.is_valid = false; // 默认置为无效
  state.position = 0;
  state.position_register = 0;
  state.status_register = 0;
  state.current_register = 0;
  state.softness_register = 0;
  state.is_moving = false;

  if (response.empty() || !modbus_utils::verifyCRC(response)) {
    return state;
  }

  auto values = modbus_utils::parseRegisterValues(response);
  if (values.size() >= 10) {
    state.is_valid = true; // 成功解析到数据，标记为有效
    // 填充 0x0000-0x0005: 力传感器数据
    state.x1_value = values[0];
    state.y1_value = values[1];
    state.z1_value = values[2];
    state.x2_value = values[3];
    state.y2_value = values[4];
    state.z2_value = values[5];
    
    // 填充 0x0006-0x0009: 状态与控制寄存器
    state.status_register = static_cast<uint16_t>(values[6]);
    state.softness_register = static_cast<uint16_t>(values[7]);
    state.position_register = static_cast<uint16_t>(values[8]);
    state.current_register = static_cast<uint16_t>(values[9]);

    // 位置寄存器 100(张开) ~ 0(闭合)
    const double range = static_cast<double>(open_register_ - closed_register_);
    double normalized = 0.0;
    if (range > 0.0) {
      normalized = (static_cast<double>(open_register_) - state.position_register) / range;
    }
    normalized = std::clamp(normalized, 0.0, 1.0);
    state.position = static_cast<uint8_t>(std::round(normalized * 255.0));

    // 状态寄存器 00/01/02/03，01/02/03 视为运动中
    state.is_moving = (state.status_register != 0x0000);
  }

  return state;
}

bool DefaultDriver::grip(uint16_t speed)
{
  if (!isConnected()) {
    return false;
  }

  // 确保处于自动模式 (Control Mode = 0x0000)
  writeSingleWithReadback(kControlModeRegister, kControlModeAuto);

  uint16_t target_speed = (speed >= 200 && speed <= 1500) ? speed : default_speed_register_;

  // 功能码0x10，起始0x000A：速度寄存器 + 命令寄存器(1=夹取)
  std::vector<uint16_t> values = {target_speed, 0x0001};
  writeMultipleWithReadback(kSpeedRegister, values);

  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  return true;
}

bool DefaultDriver::release(uint16_t speed)
{
  if (!isConnected()) {
    return false;
  }

  // 确保处于自动模式 (Control Mode = 0x0000)
  writeSingleWithReadback(kControlModeRegister, kControlModeAuto);

  uint16_t target_speed = (speed >= 200 && speed <= 1500) ? speed : default_speed_register_;

  // 功能码0x10，起始0x000A：速度寄存器 + 命令寄存器(0=松开)
  std::vector<uint16_t> values = {target_speed, 0x0000};
  writeMultipleWithReadback(kSpeedRegister, values);

  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  return true;
}

std::vector<int16_t> DefaultDriver::readFingerState(uint16_t start_addr, uint16_t count)
{
  if (!isConnected()) {
    return {};
  }

  auto frame = modbus_utils::buildReadRegistersFrame(slave_id_, start_addr, count);
  
  // Expected response size: slave_id(1) + function(1) + byte_count(1) + data(2*count) + CRC(2)
  size_t expected_size = 5 + 2 * count;
  
  auto response = writeAndRead(frame, expected_size);
  
  if (response.empty()) {
    return {};
  }
  
  if (!modbus_utils::verifyCRC(response)) {
    return {};
  }
  
  return modbus_utils::parseRegisterValues(response);
}

bool DefaultDriver::writeSingleWithReadback(uint16_t register_addr, uint16_t value)
{
  std::lock_guard<std::recursive_mutex> lock(serial_mutex_);
  const int max_retries = 3;
  for (int attempt = 0; attempt < max_retries; ++attempt) {
    auto write_frame = modbus_utils::buildWriteSingleRegisterFrame(slave_id_, register_addr, value);
    auto write_response = writeAndRead(write_frame, 8);
    if (write_response.size() != 8 || !modbus_utils::verifyCRC(write_response)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    auto read_frame = modbus_utils::buildReadRegistersFrame(slave_id_, register_addr, 1);
    auto read_response = writeAndRead(read_frame, 5 + 2);
    if (read_response.size() < 7 || !modbus_utils::verifyCRC(read_response)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    auto values = modbus_utils::parseRegisterValues(read_response);
    if (values.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    if (static_cast<uint16_t>(values.front()) == value) {
      return true;
    }
  }
  return false;
}

bool DefaultDriver::writeMultipleWithReadback(uint16_t start_addr, const std::vector<uint16_t>& values)
{
  if (values.empty()) {
    return true;
  }

  std::lock_guard<std::recursive_mutex> lock(serial_mutex_);
  const int max_retries = 3;
  for (int attempt = 0; attempt < max_retries; ++attempt) {
    auto write_frame = modbus_utils::buildWriteMultipleRegistersFrame(slave_id_, start_addr, values);
    auto write_response = writeAndRead(write_frame, 8);
    if (write_response.size() != 8 || !modbus_utils::verifyCRC(write_response) ||
        !std::equal(write_response.begin(), write_response.begin()+6, write_frame.begin())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    auto read_frame = modbus_utils::buildReadRegistersFrame(slave_id_, start_addr, static_cast<uint16_t>(values.size()));
    // A moving position target need not read back identically. Verify the Modbus
    // write acknowledgement (slave, function, start, count); read measured state separately.
    if (start_addr == kPositionModeSpeedRegister) {
      return std::equal(write_response.begin(), write_response.begin()+6, write_frame.begin());
    }
    auto read_response = writeAndRead(read_frame, 5 + 2 * values.size());
    if (read_response.empty() || !modbus_utils::verifyCRC(read_response)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    auto read_values = modbus_utils::parseRegisterValues(read_response);
    if (read_values.size() < values.size()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    bool matched = true;
    for (size_t i = 0; i < values.size(); ++i) {
      if (static_cast<uint16_t>(read_values[i]) != values[i]) {
        matched = false;
      }
    }

    if (matched) {
      return true;
    }
  }

  return false;
}

uint16_t DefaultDriver::mapPositionByteToRegister(uint8_t position_byte) const
{
  // 0(open) -> open_register_, 255(close) -> closed_register_
  const double ratio = static_cast<double>(position_byte) / 255.0;
  const double range = static_cast<double>(open_register_ - closed_register_);
  double reg_value = static_cast<double>(open_register_) - ratio * range;
  reg_value = std::clamp(reg_value, static_cast<double>(closed_register_), static_cast<double>(open_register_));
  return static_cast<uint16_t>(std::round(reg_value));
}

uint16_t DefaultDriver::mapSpeedByteToRegister(uint8_t speed_byte) const
{
  // 0 -> 200, 255 -> 1500
  const double ratio = static_cast<double>(speed_byte) / 255.0;
  double reg_value = 200.0 + ratio * (1500.0 - 200.0);
  reg_value = std::clamp(reg_value, 200.0, 1500.0);
  return static_cast<uint16_t>(std::round(reg_value));
}

uint16_t DefaultDriver::mapForceByteToPercent(uint8_t force_byte) const
{
  // 0 -> 1, 255 -> 100
  const double ratio = static_cast<double>(force_byte) / 255.0;
  double force = 1.0 + ratio * (100.0 - 1.0);
  force = std::clamp(force, 1.0, 100.0);
  return static_cast<uint16_t>(std::round(force));
}

std::vector<uint8_t> DefaultDriver::writeAndRead(
  const std::vector<uint8_t>& frame,
  size_t expected_response_size)
{
  std::lock_guard<std::recursive_mutex> lock(serial_mutex_);
  
  // Discard old bytes BEFORE transmission. Flushing after write can discard the
  // outgoing request or a fast device acknowledgement (DefaultSerial uses TCIOFLUSH).
  serial_->flush();
  size_t written = serial_->write(frame);
  if (written != frame.size()) {
    return {};
  }
  
  
  // Wait for device to process
  std::this_thread::sleep_for(kResponseDelay);
  
  // Wait for response
  auto start_time = std::chrono::steady_clock::now();
  while (serial_->available() < expected_response_size) {
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > kMaxWaitTime) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  
  if (serial_->available() < expected_response_size) {
    return {};
  }
  
  auto response = serial_->read(expected_response_size);
  if (response.size() != expected_response_size || response[0] != frame[0] || response[1] != frame[1]) return {};
  return response;
}

}  // namespace hkv_gripper_controller
