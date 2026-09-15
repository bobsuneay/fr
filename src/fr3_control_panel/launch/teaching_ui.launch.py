"""Launch the single-arm teaching UI against an already running MoveIt backend."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('fr3_control_panel')) /
                         'config' / 'panel.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(package='fr3_control_panel', executable='panel', output='screen',
             arguments=['--config', LaunchConfiguration('config')]),
    ])
