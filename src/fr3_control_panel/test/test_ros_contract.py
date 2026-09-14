"""Run on Ubuntu with ROS sourced; verifies real generated message contracts."""
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml

pytest.importorskip('rclpy')
from rclpy.serialization import serialize_message
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from fr3_control_panel.backend import Backend


@pytest.mark.parametrize('execute', [False, True])
def test_motion_goal_serializes_and_keeps_collision_scene(execute):
    cfg = yaml.safe_load((Path(__file__).parents[1]/'config/panel.yaml').read_text())
    captured = []
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = cfg['joints']
    trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.0]*6)]
    result = MoveGroup.Result()
    result.error_code.val = 1
    result.planned_trajectory = trajectory
    result.trajectory_start = RobotState()
    def action(client, goal):
        assert serialize_message(goal)
        captured.append(goal)
        return result
    dummy = SimpleNamespace(cfg=cfg, state=lambda: None, action=action, move=None,
                            display=SimpleNamespace(publish=lambda msg: serialize_message(msg)))
    Backend.motion(dummy, [.3, .1, .4, 180, 0, 90], execute)
    goal = captured[0]
    assert goal.planning_options.plan_only == (not execute)
    assert goal.request.start_state.is_diff
    assert goal.planning_options.planning_scene_diff.robot_state.is_diff
    assert not goal.planning_options.planning_scene_diff.allowed_collision_matrix.entry_names
    assert goal.request.goal_constraints[0].position_constraints[0].link_name == cfg['tcp']
