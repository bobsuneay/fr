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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <ros2_hkv_gripper/msg/gripper_registers.hpp>
#include <ros2_hkv_gripper/srv/gripper_command.hpp>
#include <ros2_hkv_gripper/srv/read_finger_state.hpp>

#include "hkv_gripper_controller/driver.hpp"
#include <memory>

namespace hkv_gripper_controller
{

/**
 * @brief Main gripper controller node (Combines control and 100Hz register monitoring)
 */
class GripperControllerNode : public rclcpp::Node
{
public:
  explicit GripperControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  ~GripperControllerNode() override;

private:
  /**
   * @brief Initialize the driver and connect to gripper
   */
  void initializeDriver();

  /**
   * @brief Timer callback to poll finger state (100Hz)
   */
  void pollFingerState();

  /**
   * @brief Service callback for gripper command
   */
  void handleGripperCommand(
    const std::shared_ptr<ros2_hkv_gripper::srv::GripperCommand::Request> request,
    std::shared_ptr<ros2_hkv_gripper::srv::GripperCommand::Response> response);

  /**
   * @brief Service callback for reading finger state
   */
  void handleReadFingerState(
    const std::shared_ptr<ros2_hkv_gripper::srv::ReadFingerState::Request> request,
    std::shared_ptr<ros2_hkv_gripper::srv::ReadFingerState::Response> response);

  // Parameters
  std::string serial_port_;
  int baud_rate_;
  double poll_rate_hz_;
  bool enable_monitor_;

  // Driver
  std::shared_ptr<Driver> driver_;

  // State cache
  std::vector<int16_t> latest_fingers_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<ros2_hkv_gripper::msg::GripperRegisters>::SharedPtr registers_pub_;
  rclcpp::Service<ros2_hkv_gripper::srv::GripperCommand>::SharedPtr cmd_srv_;
  rclcpp::Service<ros2_hkv_gripper::srv::ReadFingerState>::SharedPtr read_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hkv_gripper_controller
