"""One dual hardware bringup and one selected-arm operator panel."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    side, mode = arg('side'), arg('mode')
    panel_share = Path(get_package_share_directory('fr3_control_panel'))
    cfg = yaml.safe_load((panel_share/'config/panel.yaml').read_text())
    cfg.update(group=side+'_arm', frame='world', tcp=side+'_gripper_tcp',
               joints=[f'{side}_j{i}' for i in range(1, 7)],
               gripper_joint=side+'_gripper_joint',
               gripper_action=f'/{side}_gripper_controller/gripper_cmd',
               touch_links=[side+'_'+s for s in ('gripper_tcp', 'gripper_palm', 'left_finger', 'right_finger')])
    temp = tempfile.TemporaryDirectory(prefix='fr3_operator_')
    config = Path(temp.name)/'panel.yaml'
    config.write_text(yaml.safe_dump(cfg))
    share = Path(get_package_share_directory('fr3_dual_bolt_cell'))
    bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/bringup.launch.py')),
        launch_arguments={name: arg(name) for name in ('mode', 'hardware', 'enable_execution')}.items())
    panel = Node(package='fr3_control_panel', executable='panel',
        arguments=['--config', str(config)]+(['--mock'] if mode == 'mock' else []), output='screen')
    return [RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: temp.cleanup())), bringup, panel]


def generate_launch_description():
    share = Path(get_package_share_directory('fr3_dual_bolt_cell'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('side', default_value='left', choices=['left', 'right']),
        DeclareLaunchArgument('enable_execution', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('hardware', default_value=str(share/'config/hardware.example.yaml')),
        OpaqueFunction(function=start),
    ])
