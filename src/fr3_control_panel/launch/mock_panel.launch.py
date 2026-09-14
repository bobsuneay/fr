"""Launch the combined virtual robot, MoveIt, RViz and PyQt panel."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fr3_real_bringup'))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/mock.launch.py'))),
        Node(package='fr3_control_panel', executable='panel', arguments=['--mock'],
             output='screen'),
    ])
