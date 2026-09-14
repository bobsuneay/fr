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

#include "hkv_gripper_controller/gripper_controller_node.hpp"
#include "hkv_gripper_controller/default_driver.hpp"
#include "hkv_gripper_controller/default_serial.hpp"

namespace hkv_gripper_controller
{

GripperControllerNode::GripperControllerNode(const rclcpp::NodeOptions& options)
  : Node("gripper_controller_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 1000000);
  this->declare_parameter<double>("poll_rate_hz", 100.0);
  this->declare_parameter<bool>("enable_monitor", true);

  // Get parameters
  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  poll_rate_hz_ = this->get_parameter("poll_rate_hz").as_double();
  enable_monitor_ = this->get_parameter("enable_monitor").as_bool();

  // Initialize state cache
  latest_fingers_.resize(6, 0);

  // Initialize driver
  initializeDriver();

  // Create publisher
  state_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
    "gripper/finger_state", 10);
  
  registers_pub_ = this->create_publisher<ros2_hkv_gripper::msg::GripperRegisters>(
    "gripper/registers", 10);

  // Create services
  cmd_srv_ = this->create_service<ros2_hkv_gripper::srv::GripperCommand>(
    "gripper_command",
    std::bind(&GripperControllerNode::handleGripperCommand, this,
              std::placeholders::_1, std::placeholders::_2));

  read_srv_ = this->create_service<ros2_hkv_gripper::srv::ReadFingerState>(
    "read_finger_state",
    std::bind(&GripperControllerNode::handleReadFingerState, this,
              std::placeholders::_1, std::placeholders::_2));

  // Create timer for polling
  if (enable_monitor_ && poll_rate_hz_ > 0.0) {
    auto timer_period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&GripperControllerNode::pollFingerState, this));
    RCLCPP_INFO(this->get_logger(), "Register monitor enabled at %.1f Hz", poll_rate_hz_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Register monitor is disabled");
  }

  RCLCPP_INFO(this->get_logger(),
              "Gripper controller initialized. Port: %s, Baud rate: %d",
              serial_port_.c_str(), baud_rate_);
}

GripperControllerNode::~GripperControllerNode()
{
  if (driver_) {
    driver_->disconnect();
  }
  RCLCPP_INFO(this->get_logger(), "Gripper controller node shutting down");
}

void GripperControllerNode::initializeDriver()
{
  try {
    auto serial = std::make_shared<DefaultSerial>(serial_port_, baud_rate_, 1.0);
    driver_ = std::make_shared<DefaultDriver>(serial);
    driver_->setSlaveAddress(0x01);
    driver_->setPositionRange(100, 0);
    driver_->setDefaults(1000, 50);

    if (driver_->connect()) {
      RCLCPP_INFO(this->get_logger(),
                  "Successfully connected to serial port %s at %d baud", serial_port_.c_str(), baud_rate_);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to connect to serial port %s", serial_port_.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Error initializing driver: %s", e.what());
  }
}

void GripperControllerNode::pollFingerState()
{
  if (!driver_ || !driver_->isConnected()) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Driver not connected");
    return;
  }

  // 统一读取 0x0000 开始的 10 个保持寄存器，合并了原来 RegisterMonitorNode 的功能
  auto state = driver_->readFingerState();
  
  if (state.position_register == 0 && state.status_register == 0 && state.x1_value == 0) {
    // 可能是读取失败，返回了空结构体
    return;
  }

  // 更新最新的前 6 个力寄存器缓存，以兼容原有的 /gripper/finger_state 和 read_finger_state 服务
  latest_fingers_ = {
    state.x1_value, state.y1_value, state.z1_value,
    state.x2_value, state.y2_value, state.z2_value
  };

  auto msg = std_msgs::msg::Int16MultiArray();
  msg.data = latest_fingers_;
  state_pub_->publish(msg);

  // 发布自定义的 10 个寄存器状态结构消息
  ros2_hkv_gripper::msg::GripperRegisters reg_msg;
  reg_msg.header.stamp = this->now();
  reg_msg.header.frame_id = "gripper";
  reg_msg.x1_value = state.x1_value;
  reg_msg.y1_value = state.y1_value;
  reg_msg.z1_value = state.z1_value;
  reg_msg.x2_value = state.x2_value;
  reg_msg.y2_value = state.y2_value;
  reg_msg.z2_value = state.z2_value;
  reg_msg.status = state.status_register;
  reg_msg.softness = state.softness_register;
  reg_msg.position = state.position_register;
  reg_msg.current = state.current_register;
  registers_pub_->publish(reg_msg);
}

void GripperControllerNode::handleGripperCommand(
  const std::shared_ptr<ros2_hkv_gripper::srv::GripperCommand::Request> request,
  std::shared_ptr<ros2_hkv_gripper::srv::GripperCommand::Response> response)
{
  if (!driver_ || !driver_->isConnected()) {
    response->success = false;
    response->message = "Serial port not initialized or not connected";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  bool success = false;
  std::string action;

  if (request->command) {
    success = driver_->grip(request->speed);
    action = "grip";
  } else {
    success = driver_->release(request->speed);
    action = "release";
  }

  if (success) {
    response->success = true;
    response->message = "Successfully sent " + action + " command with speed " + std::to_string(request->speed);
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  } else {
    response->success = false;
    response->message = "Failed to send " + action + " command";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

void GripperControllerNode::handleReadFingerState(
  const std::shared_ptr<ros2_hkv_gripper::srv::ReadFingerState::Request> /*request*/,
  std::shared_ptr<ros2_hkv_gripper::srv::ReadFingerState::Response> response)
{
  if (!driver_ || !driver_->isConnected()) {
    response->fingers.clear();
    RCLCPP_WARN(this->get_logger(), "Serial port not initialized or not connected");
    return;
  }

  auto values = driver_->readFingerState(0x0000, 6);
  
  if (!values.empty()) {
    latest_fingers_ = values;
    response->fingers = values;
  } else {
    response->fingers.clear();
    RCLCPP_WARN(this->get_logger(), "Failed to read finger state");
  }
}

}  // namespace hkv_gripper_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hkv_gripper_controller::GripperControllerNode)
