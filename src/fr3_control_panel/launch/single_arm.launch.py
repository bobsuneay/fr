"""Only the UI starts initially; Connect owns one single-arm backend. No simulation task."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    arguments = ['--managed', '--config', arg('config'), '--robot-ip', arg('robot_ip'),
                 '--serial-port', arg('serial_port'), '--real-config', arg('real_config'),
                 '--cell', arg('cell')]
    if arg('mode') == 'mock':
        arguments.append('--mock')
    if arg('enable_execution') == 'true':
        arguments.append('--allow-execution')
    return [Node(package='fr3_control_panel', executable='panel', arguments=arguments, output='screen')]


def generate_launch_description():
    panel = Path(get_package_share_directory('fr3_control_panel'))
    bringup = Path(get_package_share_directory('fr3_real_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('enable_execution', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('robot_ip', default_value='192.168.58.2'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('config', default_value=str(panel/'config/panel.yaml')),
        DeclareLaunchArgument('real_config', default_value=str(bringup/'config/real.example.yaml')),
        DeclareLaunchArgument('cell', default_value=str(bringup/'config/cell.yaml')),
        OpaqueFunction(function=start),
    ])
