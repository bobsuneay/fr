// Copyright (c) 2024
// Licensed under BSD-3-Clause License

#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hkv_gripper_controller/driver.hpp"
#include "hkv_gripper_controller/driver_factory.hpp"
#include "ros2_hkv_gripper/msg/gripper_registers.hpp"

namespace hkv_gripper_controller
{

class GripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GripperHardwareInterface)

  GripperHardwareInterface();
  ~GripperHardwareInterface() override;

  // Constructor for testing
  explicit GripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory);

  // Initialization from URDF
    hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& hardware_info) override;

  // Connect to hardware
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  // Export state interfaces (position, velocity)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    // Export command interfaces (position)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Activate hardware
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  // Deactivate hardware
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // Read from hardware
  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Write to hardware
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // Driver interface
  std::unique_ptr<Driver> driver_;

  // Factory to create driver
  std::unique_ptr<DriverFactory> driver_factory_;

  // Background communication thread
  std::thread communication_thread_;
  std::atomic<bool> communication_thread_is_running_;
  void background_task();

  // 100Hz 寄存器数据发布
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ros2_hkv_gripper::msg::GripperRegisters>::SharedPtr registers_publisher_;
  rclcpp::TimerBase::SharedPtr registers_timer_;
  void publishRegisters();
  std::mutex registers_mutex_;
  FingerState latest_registers_;

  // 命令下发节流：仅在变化或保活间隔到达时发送
  uint8_t last_written_command_ = 0;
  uint8_t last_written_speed_ = 0;
  uint8_t last_written_force_ = 0;
  std::chrono::steady_clock::time_point last_write_time_{};
  std::chrono::milliseconds command_keepalive_{0};

  // 启动阶段抑制写入，直到收到与当前实际位置不同的命令
  bool suppress_writes_until_command_change_{true};
  double last_command_position_ = std::numeric_limits<double>::quiet_NaN();

  // Gripper parameters
  // Model identifier (e.g., TG-9801) for future multi-model support
  std::string gripper_model_ = "TG-9801";
  double gripper_closed_pos_ = 0.0;
  double gripper_max_speed_ = 0.0;
  double gripper_max_force_ = 0.0;

  // State variables
  double gripper_position_ = 0.0;
  double gripper_velocity_ = 0.0;
  double gripper_position_command_ = 0.0;

    // Protocol parameters
    uint16_t position_open_register_ = 100;
    uint16_t position_closed_register_ = 0;
    uint16_t position_mode_speed_register_ = 1000;  // 200~1500
    uint16_t target_force_percent_ = 50;            // 1~100

  // Atomic variables for thread-safe communication
  std::atomic<uint8_t> write_command_;
  std::atomic<uint8_t> write_force_;
  std::atomic<uint8_t> write_speed_;
  std::atomic<uint8_t> gripper_current_state_;

  // Reactivation interface
  double reactivate_gripper_cmd_ = 0.0;
  std::atomic<bool> reactivate_gripper_async_cmd_;
  double reactivate_gripper_response_ = 0.0;
  std::atomic<std::optional<bool>> reactivate_gripper_async_response_;

  static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace hkv_gripper_controller
