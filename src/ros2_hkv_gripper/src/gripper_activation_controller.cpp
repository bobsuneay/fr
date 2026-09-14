// Copyright (c) 2024
// Licensed under BSD-3-Clause License

#include <chrono>
#include <cmath>
#include <optional>
#include <thread>
#include <type_traits>

#include "hkv_gripper_controller/gripper_activation_controller.hpp"

namespace hkv_gripper_controller
{

namespace
{
template <typename CommandInterface>
bool set_command_value(CommandInterface& iface, double value)
{
  using set_ret_t = decltype(std::declval<CommandInterface&>().set_value(std::declval<double>()));
  if constexpr (std::is_same_v<set_ret_t, bool>) {
    return iface.set_value(value);
  } else {
    iface.set_value(value);
    return true;
  }
}

template <typename CommandInterface>
struct has_get_optional
{
  template <typename T>
  static auto test(int) -> decltype(std::declval<T&>().get_optional(), std::true_type{});
  template <typename>
  static std::false_type test(...);
  static constexpr bool value = decltype(test<CommandInterface>(0))::value;
};

template <typename CommandInterface>
std::optional<double> get_optional_value(CommandInterface& iface)
{
  if constexpr (has_get_optional<CommandInterface>::value) {
    return iface.get_optional();
  } else {
    return std::optional<double>{iface.get_value()};
  }
}
}  // namespace

controller_interface::InterfaceConfiguration 
GripperActivationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("reactivate_gripper/reactivate_gripper_cmd");
  config.names.emplace_back("reactivate_gripper/reactivate_gripper_response");

  return config;
}

controller_interface::InterfaceConfiguration 
GripperActivationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::return_type GripperActivationController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  return controller_interface::return_type::OK;
}

GripperActivationController::CallbackReturn 
GripperActivationController::on_init()
{
  return CallbackReturn::SUCCESS;
}

GripperActivationController::CallbackReturn 
GripperActivationController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Create service for gripper reactivation
  reactivate_gripper_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/reactivate_gripper",
      std::bind(&GripperActivationController::reactivateGripper, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_node()->get_logger(), "GripperActivationController activated");
  return CallbackReturn::SUCCESS;
}

GripperActivationController::CallbackReturn 
GripperActivationController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  reactivate_gripper_srv_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "GripperActivationController deactivated");
  return CallbackReturn::SUCCESS;
}

bool GripperActivationController::reactivateGripper(
    std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  RCLCPP_INFO(get_node()->get_logger(), "Reactivating gripper...");

  // Send reactivation command (set_value returns void across Humble/Jazzy)
  if (!set_command_value(command_interfaces_[CommandInterfaces::REACTIVATE_GRIPPER_CMD], 1.0))
  {
    resp->success = false;
    resp->message = "Failed to set reactivation command";
    return false;
  }

  // Wait for response
  auto start_time = get_node()->now();
  while ((get_node()->now() - start_time).seconds() < ASYNC_WAITING)
  {
    auto response_opt = get_optional_value(command_interfaces_[CommandInterfaces::REACTIVATE_GRIPPER_RESPONSE]);
    if (response_opt.has_value())
    {
      double response_value = response_opt.value();
      if (!std::isnan(response_value) && response_value > 0.0)
      {
        resp->success = true;
        resp->message = "Gripper reactivated successfully";
        RCLCPP_INFO(get_node()->get_logger(), "Gripper reactivated successfully");
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  resp->success = false;
  resp->message = "Gripper reactivation timeout";
  RCLCPP_WARN(get_node()->get_logger(), "Gripper reactivation timeout");
  return false;
}

}  // namespace hkv_gripper_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hkv_gripper_controller::GripperActivationController, 
                       controller_interface::ControllerInterface)
