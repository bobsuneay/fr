// Copyright (c) 2024
// Licensed under BSD-3-Clause License

#include "hkv_gripper_controller/gripper_hardware_interface.hpp"
#include "hkv_gripper_controller/default_driver_factory.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>

namespace hkv_gripper_controller
{

const auto kLogger = rclcpp::get_logger("GripperHardwareInterface");

// Constants for gripper range (0-255)
constexpr uint8_t kGripperMinPos = 0;
constexpr uint8_t kGripperMaxPos = 255;
constexpr uint8_t kGripperRange = kGripperMaxPos - kGripperMinPos;

// EG-9801 register defaults
constexpr uint16_t kDefaultOpenRegister = 100;
constexpr uint16_t kDefaultClosedRegister = 0;
constexpr uint16_t kDefaultPositionModeSpeedReg = 1000;  // 200~1500
constexpr uint16_t kDefaultForcePercent = 50;             // 1~100

// Communication loop period
constexpr auto kGripperCommsLoopPeriod = std::chrono::milliseconds{ 10 };

GripperHardwareInterface::GripperHardwareInterface()
  : communication_thread_is_running_(false)
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();
}

GripperHardwareInterface::~GripperHardwareInterface()
{
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }
}

GripperHardwareInterface::GripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_(std::move(driver_factory))
  , communication_thread_is_running_(false)
{
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& hardware_info)
{
  RCLCPP_DEBUG(kLogger, "on_init");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters
  // Optional gripper model identifier
  auto it_model = info_.hardware_parameters.find("gripper_model");
  if (it_model != info_.hardware_parameters.end()) {
    gripper_model_ = it_model->second;
  }
  RCLCPP_INFO(rclcpp::get_logger("GripperHardwareInterface"), "Gripper model: %s", gripper_model_.c_str());

  auto it = info_.hardware_parameters.find("gripper_closed_position");
  if (it != info_.hardware_parameters.end())
  {
    gripper_closed_pos_ = std::stod(it->second);
  }
  else
  {
    gripper_closed_pos_ = 0.1;  // Default closed position in meters (0=open,0.1=closed)
  }

  joint_closed_ = gripper_closed_pos_;
  auto endpoint = info_.hardware_parameters.find("joint_position_open");
  if (endpoint != info_.hardware_parameters.end()) joint_open_ = std::stod(endpoint->second);
  endpoint = info_.hardware_parameters.find("joint_position_closed");
  if (endpoint != info_.hardware_parameters.end()) joint_closed_ = std::stod(endpoint->second);
  endpoint = info_.hardware_parameters.find("feedback_timeout");
  if (endpoint != info_.hardware_parameters.end()) feedback_timeout_ = std::stod(endpoint->second);
  if (!std::isfinite(joint_open_) || !std::isfinite(joint_closed_) ||
      std::abs(joint_open_-joint_closed_) < 1e-6 || !std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Register parameters from EG-9801
  it = info_.hardware_parameters.find("position_open_register");
  position_open_register_ = (it != info_.hardware_parameters.end())
      ? static_cast<uint16_t>(std::stoi(it->second))
      : kDefaultOpenRegister;

  it = info_.hardware_parameters.find("position_closed_register");
  position_closed_register_ = (it != info_.hardware_parameters.end())
      ? static_cast<uint16_t>(std::stoi(it->second))
      : kDefaultClosedRegister;

  // Clamp to valid ranges (open >= closed)
  if (position_open_register_ <= position_closed_register_)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  it = info_.hardware_parameters.find("position_mode_speed_register");
  position_mode_speed_register_ = (it != info_.hardware_parameters.end())
      ? static_cast<uint16_t>(std::stoi(it->second))
      : kDefaultPositionModeSpeedReg;
  position_mode_speed_register_ = static_cast<uint16_t>(std::clamp<int>(position_mode_speed_register_, 200, 1500));

  it = info_.hardware_parameters.find("target_force_percent");
  target_force_percent_ = (it != info_.hardware_parameters.end())
      ? static_cast<uint16_t>(std::stoi(it->second))
      : kDefaultForcePercent;
  target_force_percent_ = static_cast<uint16_t>(std::clamp<int>(target_force_percent_, 1, 100));

  // 可选：命令保活间隔（毫秒），默认0=关闭，仅在变化时写入
  it = info_.hardware_parameters.find("command_keepalive_ms");
  if (it != info_.hardware_parameters.end())
  {
    int keepalive = std::stoi(it->second);
    if (keepalive < 0) keepalive = 0;
    if (keepalive != 0) {
      RCLCPP_ERROR(kLogger, "Keepalive is disabled; command updates have one writer");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Precompute fixed speed/force bytes for driver mapping
  auto speed_to_byte = [](uint16_t speed_reg) {
    double ratio = (static_cast<double>(speed_reg) - 200.0) / (1500.0 - 200.0);
    ratio = std::clamp(ratio, 0.0, 1.0);
    return static_cast<uint8_t>(std::round(ratio * 255.0));
  };

  auto force_to_byte = [](uint16_t force_percent) {
    double ratio = (static_cast<double>(force_percent) - 1.0) / (100.0 - 1.0);
    ratio = std::clamp(ratio, 0.0, 1.0);
    return static_cast<uint8_t>(std::round(ratio * 255.0));
  };

  write_speed_.store(speed_to_byte(position_mode_speed_register_));
  write_force_.store(force_to_byte(target_force_percent_));

  // Initialize state variables
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();
  reactivate_gripper_cmd_ = NO_NEW_CMD_;
  reactivate_gripper_async_cmd_.store(false);

  // Validate joint configuration
  if (info_.joints.empty())
  {
    RCLCPP_FATAL(kLogger, "No joints defined in hardware interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const hardware_interface::ComponentInfo& joint = info_.joints[0];

  // Validate command interfaces
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu command interfaces. Expected 1 (position).",
                 joint.name.c_str(), joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has '%s' command interface. Expected '%s'.",
                 joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                 hardware_interface::HW_IF_POSITION);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate state interfaces
  if (joint.state_interfaces.size() != 2)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu state interfaces. Expected 2 (position, velocity).",
                 joint.name.c_str(), joint.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto& state_interface : joint.state_interfaces)
  {
    if (state_interface.name != hardware_interface::HW_IF_POSITION &&
        state_interface.name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(kLogger, "Joint '%s' has invalid state interface '%s'.",
                   joint.name.c_str(), state_interface.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Create driver
  try
  {
    driver_ = driver_factory_->create(info_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to create driver: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");

  if (hardware_interface::SystemInterface::on_configure(previous_state) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    bool connected = driver_->connect();
    if (!connected)
    {
      RCLCPP_ERROR(kLogger, "Cannot connect to gripper");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 应用寄存器范围与默认速度/力配置
    driver_->setPositionRange(position_open_register_, position_closed_register_);
    driver_->setDefaults(position_mode_speed_register_, target_force_percent_);
    
    // 初始化 ROS2 node 和 publisher
    node_ = rclcpp::Node::make_shared("gripper_hardware_interface");
    registers_publisher_ = node_->create_publisher<ros2_hkv_gripper::msg::GripperRegisters>(
        "/gripper_registers", 10);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot configure gripper: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_state_interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(
          info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(
          info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Position command interface
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_command_));

  // Reactivation command interfaces
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
          "reactivate_gripper", "reactivate_gripper_cmd", &reactivate_gripper_cmd_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
          "reactivate_gripper", "reactivate_gripper_response", &reactivate_gripper_response_));

  return command_interfaces;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(kLogger, "on_activate");

  // Set default values
  if (std::isnan(gripper_position_))
  {
    gripper_position_ = 0;
    gripper_velocity_ = 0;
    gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();
  }

  // Activate gripper
  try
  {
    // Preserve the physical holding state. Activation must not send release/reset.

    // 启动前先读取一次当前寄存器位置，避免初始化时发生移动
    try {
      auto state = driver_->readFingerState();
      if (!state.is_valid) throw std::runtime_error("Initial register read invalid");
      if (state.position_register < position_closed_register_ || state.position_register > position_open_register_)
        throw std::runtime_error("Initial position outside calibrated register range");
      {
        std::lock_guard<std::mutex> lock(registers_mutex_);
        latest_registers_ = state;
        last_feedback_ = std::chrono::steady_clock::now();
      }
      // 将当前实际位置同步到命令值，防止初始化写入造成移动
      const double range = static_cast<double>(position_open_register_ - position_closed_register_);
      double normalized_closed = 0.0;
      if (range > 0.0) {
        normalized_closed = static_cast<double>(position_open_register_ - state.position_register) / range;
      }
      normalized_closed = std::clamp(normalized_closed, 0.0, 1.0);
      gripper_position_ = joint_open_ + (joint_closed_-joint_open_) * normalized_closed;
      gripper_velocity_ = 0.0;
      last_read_ = std::chrono::steady_clock::now();
      
      gripper_position_command_ = gripper_position_;
      last_command_position_ = gripper_position_;
      suppress_writes_until_command_change_ = true;
      
      const uint8_t cmd = static_cast<uint8_t>(std::round(normalized_closed * 255.0));
      write_command_.store(cmd);
      last_written_command_ = cmd;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(kLogger, "Failed to sync initial position: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    last_write_time_ = std::chrono::steady_clock::now();
    write_fault_.store(false);
    // One worker owns serial I/O; never block the arm's controller_manager in write().
    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread([this] { this->background_task(); });

    // 启动 100Hz 寄存器数据发布定时器
    // Only the successful hardware polling path publishes fresh register samples.

    // 初始化命令节流缓存
    last_written_speed_ = write_speed_.load();
    last_written_force_ = write_force_.load();
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to activate gripper: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Gripper successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");

  // 停止定时器
  if (registers_timer_) {
    registers_timer_->cancel();
    registers_timer_.reset();
  }

  // Stop communication thread
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }

  try
  {
    driver_->disconnect();  // No automatic jaw release on ROS shutdown.
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to deactivate gripper: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Gripper successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GripperHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // 从缓存读取位置数据，基于寄存器 0x0008 的实时值
  // position_register 范围：0~100，映射到关节位置 0~gripper_closed_pos_ 米（默认 0.1m）
  if (write_fault_.load()) return hardware_interface::return_type::ERROR;
  uint16_t position_register = 0;
  {
    std::lock_guard<std::mutex> lock(registers_mutex_);
    position_register = latest_registers_.position_register;
    if (position_register < position_closed_register_ || position_register > position_open_register_)
      return hardware_interface::return_type::ERROR;
    if (!latest_registers_.is_valid || std::chrono::duration<double>(
        std::chrono::steady_clock::now()-last_feedback_).count() > feedback_timeout_) {
      return hardware_interface::return_type::ERROR;
    }
  }
  
  // Calculate gap distance: 
  // position_register == open_register_ -> gap = 0.0 (0.0 in ros2_control means open)
  // position_register == closed_register_ -> gap = gripper_closed_pos_ (0.1 in ros2_control means closed)
  const double range = static_cast<double>(position_open_register_ - position_closed_register_);
  double normalized_closed = 0.0;
  if (range > 0.0) {
    normalized_closed = static_cast<double>(position_open_register_ - position_register) / range;
  }
  normalized_closed = std::clamp(normalized_closed, 0.0, 1.0);
  const auto now = std::chrono::steady_clock::now();
  const double next = joint_open_ + (joint_closed_-joint_open_) * normalized_closed;
  const double dt = std::chrono::duration<double>(now-last_read_).count();
  gripper_velocity_ = dt > 0 ? (next-gripper_position_)/dt : 0.0;
  gripper_position_ = next;
  last_read_ = now;

  // Handle reactivation request
  if (!std::isnan(reactivate_gripper_cmd_))
  {
    RCLCPP_INFO(kLogger, "Sending gripper reactivation request");
    reactivate_gripper_async_cmd_.store(true);
    reactivate_gripper_cmd_ = NO_NEW_CMD_;
  }

  if (reactivate_gripper_async_response_.load().has_value())
  {
    reactivate_gripper_response_ = reactivate_gripper_async_response_.load().value();
    reactivate_gripper_async_response_.store(std::nullopt);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // Convert from joint position to gripper command byte (0-255)
  // 0.0 (open) -> 0 (open register)
  // gripper_closed_pos_ (closed) -> 255 (closed register)
  if (std::isnan(gripper_position_command_)) {
    return hardware_interface::return_type::OK;
  }
  if (!std::isfinite(gripper_position_command_) ||
      gripper_position_command_ < std::min(joint_open_, joint_closed_)-1e-8 ||
      gripper_position_command_ > std::max(joint_open_, joint_closed_)+1e-8) {
    return hardware_interface::return_type::ERROR;
  }
  double normalized_closed = (gripper_position_command_-joint_open_) / (joint_closed_-joint_open_);
  normalized_closed = std::clamp(normalized_closed, 0.0, 1.0);
  
  double gripper_pos = normalized_closed * 255.0;
  const uint8_t cmd = static_cast<uint8_t>(std::round(gripper_pos));
  if (write_fault_.load()) return hardware_interface::return_type::ERROR;
  if (suppress_writes_until_command_change_) {
    if (std::abs(gripper_position_command_-last_command_position_) < 1e-6) {
      return hardware_interface::return_type::OK;
    }
    suppress_writes_until_command_change_ = false;
  }
  write_command_.store(cmd);
  last_command_position_ = gripper_position_command_;

  return hardware_interface::return_type::OK;
}

void GripperHardwareInterface::background_task()
{
  while (communication_thread_is_running_.load())
  {
    try
    {
      // Handle reactivation request
      if (reactivate_gripper_async_cmd_.load())
      {
        reactivate_gripper_async_cmd_.store(false);
        reactivate_gripper_async_response_.store(false);
      }

      const uint8_t pending = write_command_.load();
      if (pending != last_written_command_ && !write_fault_.load()) {
        try {
          driver_->grip(pending, write_speed_.load(), write_force_.load());
          last_written_command_ = pending;
        } catch (const std::exception& e) {
          write_fault_.store(true);
          RCLCPP_ERROR(kLogger, "Gripper command failed: %s", e.what());
        }
      }

      // Read state from gripper
      auto state = driver_->readFingerState();
      
      // 如果读取失败（例如由于并发写入阻塞导致超时或 CRC 错误），直接跳过本次更新
      if (!state.is_valid) {
        std::this_thread::sleep_for(kGripperCommsLoopPeriod);
        continue;
      }

      gripper_current_state_.store(state.position);

      // 保存完整的寄存器数据供发布使用
      {
        std::lock_guard<std::mutex> lock(registers_mutex_);
        latest_registers_ = state;
        last_feedback_ = std::chrono::steady_clock::now();
      }

      // 直接在后台线程以 ~100Hz 发布寄存器数据
      if (registers_publisher_) {
        ros2_hkv_gripper::msg::GripperRegisters msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = "gripper";
        msg.x1_value = state.x1_value;
        msg.y1_value = state.y1_value;
        msg.z1_value = state.z1_value;
        msg.x2_value = state.x2_value;
        msg.y2_value = state.y2_value;
        msg.z2_value = state.z2_value;
        msg.status = state.status_register;
        msg.softness = state.softness_register;
        msg.position = state.position_register;
        msg.current = state.current_register;
        registers_publisher_->publish(msg);
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(kLogger, "Background task error: %s", e.what());
    }

    std::this_thread::sleep_for(kGripperCommsLoopPeriod);
  }
}

void GripperHardwareInterface::publishRegisters()
{
  ros2_hkv_gripper::msg::GripperRegisters msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "gripper";
  
  // 从缓存读取最新寄存器数据
  {
    std::lock_guard<std::mutex> lock(registers_mutex_);
    msg.x1_value = latest_registers_.x1_value;
    msg.y1_value = latest_registers_.y1_value;
    msg.z1_value = latest_registers_.z1_value;
    msg.x2_value = latest_registers_.x2_value;
    msg.y2_value = latest_registers_.y2_value;
    msg.z2_value = latest_registers_.z2_value;
    msg.status = latest_registers_.status_register;
    msg.softness = latest_registers_.softness_register;
    msg.position = latest_registers_.position_register;
    msg.current = latest_registers_.current_register;
  }
  
  registers_publisher_->publish(msg);
  
  // Spin once to process callbacks
  rclcpp::spin_some(node_);
}

}  // namespace hkv_gripper_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hkv_gripper_controller::GripperHardwareInterface, hardware_interface::SystemInterface)
