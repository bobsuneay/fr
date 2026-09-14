"""Launch gripper controller with ros2_control"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baud rate for serial communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware (mock) instead of real hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "position_mode_speed_register",
            default_value="1000",
            description="TG-9801 位置模式速度寄存器值 200~1500",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "target_force_percent",
            default_value="50",
            description="TG-9801 目标夹取力(1~100)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz for visualization",
        )
    )

    # Initialize arguments
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    position_mode_speed_register = LaunchConfiguration("position_mode_speed_register")
    target_force_percent = LaunchConfiguration("target_force_percent")

    # Get URDF via xacro
    pkg_share = FindPackageShare("ros2_hkv_gripper")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "gripper.urdf.xacro"]),
            " ",
            "serial_port:=",
            serial_port,
            " ",
            "baud_rate:=",
            baud_rate,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "position_mode_speed_register:=",
            position_mode_speed_register,
            " ",
            "target_force_percent:=",
            target_force_percent,
            " ",
            "gripper_model:=",
            LaunchConfiguration("gripper_model"),
        ]
    )
    # Add gripper_model argument (default TG-9801)
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_model",
            default_value="TG-9801",
            description="Gripper model identifier (e.g., TG-9801)",
        )
    )

    robot_description = {"robot_description": robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [pkg_share, "config", "gripper_controllers.yaml"]
    )

    # ros2_control_node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Gripper Controller Spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tg9801_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Gripper Activation Controller Spawner
    gripper_activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tg9801_activation_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay controller spawners after control_node starts
    delay_joint_state_broadcaster_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    delay_gripper_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[gripper_controller_spawner],
                )
            ],
        )
    )

    delay_activation_controller_after_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gripper_controller_spawner,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[gripper_activation_controller_spawner],
                )
            ],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        delay_joint_state_broadcaster_after_control_node,
        delay_gripper_controller_after_joint_state_broadcaster,
        delay_activation_controller_after_gripper_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)
