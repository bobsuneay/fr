// Copyright (c) 2024
// Licensed under BSD-3-Clause License

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hkv_gripper_controller
{

/**
 * Controller for gripper activation/reactivation.
 * Provides a service to trigger gripper reactivation (useful for re-calibration).
 */
class GripperActivationController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  CallbackReturn on_init() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  bool reactivateGripper(
      std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr resp);

  static constexpr double ASYNC_WAITING = 2.0;
  
  enum CommandInterfaces
  {
    REACTIVATE_GRIPPER_CMD,
    REACTIVATE_GRIPPER_RESPONSE
  };

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reactivate_gripper_srv_;
};

}  // namespace hkv_gripper_controller
