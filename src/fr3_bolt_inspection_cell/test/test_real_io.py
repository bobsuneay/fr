"""Real state-machine decisions, with transport replaced only (no hardware claim)."""
import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import numpy as np
import pytest

SHARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SHARE))


@pytest.fixture
def real_io(monkeypatch):
    for name in ('rclpy', 'rclpy.action', 'rclpy.duration', 'rclpy.time', 'rclpy.qos',
                 'action_msgs.msg', 'builtin_interfaces.msg', 'control_msgs.action',
                 'geometry_msgs.msg', 'moveit_msgs.action', 'moveit_msgs.msg',
                 'moveit_msgs.srv', 'shape_msgs.msg', 'std_srvs.srv',
                 'gazebo_msgs.msg', 'gazebo_msgs.srv', 'trajectory_msgs.msg', 'ros2_hkv_gripper.msg'):
        monkeypatch.setitem(sys.modules, name, MagicMock())
    spec = importlib.util.spec_from_file_location('fr3_bolt_inspection_cell.ros_io', SHARE/'fr3_bolt_inspection_cell/ros_io.py')
    base = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, base)
    spec.loader.exec_module(base)
    spec = importlib.util.spec_from_file_location('fr3_bolt_inspection_cell._test_real_io', SHARE/'fr3_bolt_inspection_cell/real_io.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    io = module.RealIO.__new__(module.RealIO)
    io.n = NS(stop_event=threading.Event(), data_lock=threading.Lock(),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
    io.c = {'close_width': .004, 'center_tolerance': .003}
    io.real = {'opening_tolerance': .0005, 'gripper_timeout': .08, 'contact_seconds': .02,
               'feedback_timeout': .5, 'left': {'finger_closed': 0., 'finger_open': .0175},
               'right': {'finger_closed': 0., 'finger_open': .0175}}
    io.owner_side = ''
    io.possible_hold, io.confirmed = set(), set()
    io.monitoring = False
    io.fingers = {'left': object(), 'right': object()}
    io.finger_position = lambda side: .0175
    io.has_contact = lambda side: False
    io.object_pose = lambda: np.eye(4)
    io.check = lambda: None
    io.action = MagicMock(return_value=NS(reached_goal=True, stalled=False))
    module.GripperCommand.Goal = lambda: NS(command=NS())
    return module, io


def test_close_success_does_not_claim_ownership_and_timeout_latches(real_io):
    _, io = real_io
    io.gripper('right', .004)
    assert io.owner_side == ''
    with pytest.raises(RuntimeError, match='uncertain'):
        io.grasp_owner()
    assert io.action.call_args.args[1].command.position == .002
    io.action.side_effect = TimeoutError('late acceptance')
    with pytest.raises(TimeoutError):
        io.gripper('left', .004)
    assert io.possible_hold == {'left', 'right'}


def test_donor_cannot_open_without_verified_receiver(real_io):
    _, io = real_io
    io.possible_hold = {'right'}
    io.owner_side = 'right'
    with pytest.raises(RuntimeError):
        io.gripper('right', .035)
    io.action.assert_not_called()
    io.owner_side = 'left'
    with pytest.raises(RuntimeError):
        io.gripper('right', .035)
    io.has_contact = lambda side: side == 'left'
    io.gripper('right', .035)
    assert not io.possible_hold


def test_lost_contact_prevents_handover_even_after_previous_confirmation(real_io):
    _, io = real_io
    io.owner_side = 'left'
    with pytest.raises(RuntimeError, match='uncertain'):
        io.grasp_owner()


def test_repeated_single_contact_sample_is_not_sustained_evidence(real_io):
    _, io = real_io
    io.has_contact = lambda side: True
    io.registers = {'right': (object(), 1., time.monotonic())}
    with pytest.raises(RuntimeError, match='Sustained'):
        io.assisted_grasp('right')
    assert not io.confirmed


def test_multiple_fresh_contact_samples_confirm_receiver_before_release(real_io):
    _, io = real_io
    io.possible_hold = {'right', 'left'}
    io.owner_side = 'right'
    io.registers = {'left': (object(), 1., time.monotonic())}
    def good(side):
        io.registers['left'] = (object(), io.registers['left'][1]+.01, time.monotonic())
        return True
    io.has_contact = good
    io.assisted_grasp('left')
    assert io.owner_side == 'left' and 'left' in io.confirmed
    io.action.assert_not_called()  # confirmation itself never moves or opens jaws
    io.gripper('right', .035)
    assert 'right' not in io.possible_hold


def test_tracker_pose_is_independent_of_robot_or_attachment(real_io):
    module, io = real_io
    actual = np.eye(4)
    actual[:3, 3] = [.5, -.2, .73]
    io.object_pose = lambda: actual
    first = io.truth()
    actual[0, 3] += .01
    assert io.truth()[0, 3]-first[0, 3] == pytest.approx(.01)
    assert np.allclose(first[:3, 2], [1, 0, 0])


def test_real_constructor_has_no_gazebo_clients(real_io):
    module, _ = real_io
    import yaml
    cfg = yaml.safe_load((SHARE/'config/real_feedback.example.yaml').read_text())
    cfg['calibrated'] = True
    for side in ('left', 'right'):
        cfg[side].update(contact_min=10, contact_max=100)
    node = MagicMock()
    node.cfg = {'real': cfg}
    node.get_parameter.return_value.value = 'real'
    module.RealIO(node)
    names = [call.args[1] for call in node.create_client.call_args_list]
    assert names and not any('/sim/' in name or 'gazebo' in name for name in names)
