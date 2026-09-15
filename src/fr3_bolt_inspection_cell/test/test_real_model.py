from pathlib import Path
import sys
import yaml

SHARE = Path(__file__).resolve().parents[1]
BASE = SHARE.parent/'fr3_dual_bolt_cell'
sys.path[:0] = [str(SHARE), str(BASE)]
from fr3_bolt_inspection_cell.real_model import configure_real_model, real_controllers
from fr3_bolt_inspection_cell.model import augment
from fr3_bolt_inspection_cell.geometry import fingertip_geometry
from fr3_dual_bolt_cell.model import build_model


def test_real_model_exports_only_twelve_axes_and_two_calibrated_grippers():
    arms = yaml.safe_load((SHARE/'config/arms.yaml').read_text())
    cfg = yaml.safe_load((SHARE/'config/inspection.yaml').read_text())
    cfg['real'] = yaml.safe_load((SHARE/'config/real_feedback.example.yaml').read_text())
    cfg['real']['calibrated'] = True
    for side in ('left', 'right'):
        cfg['real'][side].update(contact_min=10, contact_max=100)
    hw = yaml.safe_load((BASE/'config/hardware.example.yaml').read_text())
    hw['commissioned'] = True
    for side, ip, port in [('left', '192.168.1.2', '/dev/left'), ('right', '192.168.1.3', '/dev/right')]:
        hw[side].update(robot_ip=ip, serial_port=port)
    root = configure_real_model(augment(build_model(BASE, SHARE/'config/scene.yaml', arms, 'real', hardware=hw), cfg, False), cfg)
    assert not root.findall('gazebo')
    assert len(root.findall('ros2_control')) == 4
    commands = root.findall('ros2_control/joint/command_interface')
    assert len(commands) == 14
    for side in ('left', 'right'):
        system = root.find(f"ros2_control[@name='{side}_gripper_system']")
        assert [j.get('name') for j in system.findall('joint')] == [side+'_left_finger_joint']
        assert system.findtext("hardware/param[@name='joint_position_open']") == '0.0175'
        assert not system.findall('joint/state_interface/param')
        assert root.find(f"joint[@name='{side}_right_finger_joint']/mimic").get('joint') == side+'_left_finger_joint'
        assert len(fingertip_geometry(root, side, lambda uri: BASE/uri.split('fr3_dual_bolt_cell/')[1])) == 2
        control = real_controllers(side)
        assert control[side+'_gripper_controller']['ros__parameters']['joint'] == side+'_left_finger_joint'
        assert control[side+'_controller_manager']['ros__parameters']['update_rate'] == 125
