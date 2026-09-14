from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明可配置参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for gripper'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    poll_rate_arg = DeclareLaunchArgument(
        'poll_rate_hz',
        default_value='100.0',
        description='Polling rate in Hz'
    )
    
    enable_monitor_arg = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Enable automatic register monitoring at poll_rate_hz'
    )
    
    # 创建节点
    gripper_node = Node(
        package='ros2_hkv_gripper',
        executable='ros2_hkv_gripper_node',
        name='gripper_controller_node',
        parameters=[
            {'serial_port': LaunchConfiguration('serial_port')},
            {'baud_rate': LaunchConfiguration('baud_rate')},
            {'poll_rate_hz': LaunchConfiguration('poll_rate_hz')},
            {'enable_monitor': LaunchConfiguration('enable_monitor')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        poll_rate_arg,
        enable_monitor_arg,
        gripper_node
    ])