#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware {

FairinoHardwareInterface::~FairinoHardwareInterface()
{
    try { disconnect(); } catch (...) { /* Destructors must not throw. */ }
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& sysinfo)
{
    if (hardware_interface::SystemInterface::on_init(sysinfo) !=
        hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
    // dual_cell_adapter_v1: no fallback to a compiled robot IP.
    if (info_.joints.size() != 6) return hardware_interface::CallbackReturn::ERROR;
    const auto ip = info_.hardware_parameters.find("robot_ip");
    if (ip == info_.hardware_parameters.end() || ip->second.empty())
        return hardware_interface::CallbackReturn::ERROR;
    _controller_ip = ip->second;
    for (const auto& joint : info_.joints) {
        if (joint.command_interfaces.size() != 1 || joint.state_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> result;
    for (size_t i = 0; i < info_.joints.size(); ++i)
        result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]);
    return result;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> result;
    for (size_t i = 0; i < info_.joints.size(); ++i)
        result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]);
    return result;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State&)
{
    using namespace std::chrono_literals;
    disconnect();
    _ptr_robot = std::make_unique<FRRobot>();
    feedback_watchdog_.reset();
    if (_ptr_robot->RPC(_controller_ip.c_str()) != 0) {
        disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(200ms);
    // Require changing controller frames before entering servo mode. A cached packet
    // with a freshly generated ROS timestamp is not sufficient.
    bool live = read_feedback();
    for (int sample = 0; live && sample < 6; ++sample) {
        rclcpp::sleep_for(20ms);
        live = read_feedback();
    }
    if (!live) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "No healthy live controller feedback");
        disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }
    for (int j = 0; j < 6; ++j) {
        _jnt_position_command[j] = _jnt_position_state[j];
    }
    // The operator commissions and enables the controller. Never ResetAllError or RobotEnable.
    if (_ptr_robot->ServoMoveStart() != 0) {
        disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }
    faulted_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

void FairinoHardwareInterface::disconnect()
{
    if (_ptr_robot) {
        if (!faulted_) {
            _ptr_robot->StopMotion();
            _ptr_robot->ServoMoveEnd();
        }
        _ptr_robot->CloseRPC();
        _ptr_robot.reset();
    }
    faulted_ = true;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
    disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool FairinoHardwareInterface::read_feedback()
{
    if (!_ptr_robot) return false;
    ROBOT_STATE_PKG packet{};
    const bool received = _ptr_robot->GetRobotRealTimeState(&packet) == 0;
    const double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const bool healthy = received && packet.frame_head == 0x5A5A &&
        packet.main_code == 0 && packet.sub_code == 0 &&
        packet.EmergencyStop == 0 && packet.collisionState == 0 &&
        std::all_of(packet.jt_cur_pos, packet.jt_cur_pos + 6,
                    [](double value) { return std::isfinite(value); });
    if (!feedback_watchdog_.observe(packet.frame_cnt, healthy, seconds)) return false;
    for (int i = 0; i < 6; ++i) {
        _jnt_position_state[i] = packet.jt_cur_pos[i] / 180.0 * M_PI;
    }
    return true;
}

hardware_interface::return_type FairinoHardwareInterface::fail_stop()
{
    if (!faulted_ && _ptr_robot) {
        faulted_ = true;
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "Hardware fault latched; requesting stop, reactivation required");
        _ptr_robot->StopMotion();
        _ptr_robot->ServoMoveEnd();
    }
    return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (faulted_ || !read_feedback()) return fail_stop();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (faulted_ || !_ptr_robot) return hardware_interface::return_type::ERROR;
    if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[6],
                    [](double c) { return !std::isfinite(c); })) return fail_stop();
    JointPos cmd{};
    ExaxisPos extcmd{0, 0, 0, 0};
    for (int j = 0; j < 6; ++j) cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180.0;
    const int code = _ptr_robot->ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);
    if (code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ failed: %d", code);
        return fail_stop();
    }
    return hardware_interface::return_type::OK;
}

}  // namespace fairino_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
