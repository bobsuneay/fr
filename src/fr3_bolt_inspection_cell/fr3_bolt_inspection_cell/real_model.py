"""Keep inspection geometry; adapt only the real control boundary."""
import xml.etree.ElementTree as ET
from .real_contract import validate_real
from .model import linked_controllers


def configure_real_model(root, cfg):
    real = validate_real(cfg['real'])
    for side in ('left', 'right'):
        master, follower = side+'_left_finger_joint', side+'_right_finger_joint'
        system = next(s for s in root.findall('ros2_control') if s.find(f"joint[@name='{master}']") is not None)
        for joint in list(system.findall('joint')):
            if joint.get('name') != master:
                system.remove(joint)
        joint = system.find('joint')
        for param in list(joint.iter('param')):
            for parent in joint.iter():
                if param in list(parent):
                    parent.remove(param)
        if joint.find("state_interface[@name='velocity']") is None:
            ET.SubElement(joint, 'state_interface', name='velocity')
        hw = system.find('hardware')
        for key, value in [('joint_position_open', real[side]['finger_open']),
                           ('joint_position_closed', real[side]['finger_closed']),
                           ('feedback_timeout', real['feedback_timeout'])]:
            ET.SubElement(hw, 'param', name=key).text = str(value)
        for name in (master, follower):
            limit = root.find(f"joint[@name='{name}']/limit")
            limit.set('lower', str(real[side]['finger_closed']))
            limit.set('upper', str(real[side]['finger_open']))
    # Camera frames are retained, but no Gazebo plugin or simulator sensor is loaded.
    for node in list(root.findall('gazebo')):
        root.remove(node)
    return root


def real_controllers(side):
    result = linked_controllers('real', side)
    name = side+'_gripper_controller'
    result[side+'_controller_manager']['ros__parameters'][name]['type'] = 'position_controllers/GripperActionController'
    result[name] = {'ros__parameters': {
        'joint': side+'_left_finger_joint', 'goal_tolerance': .0005,
        'max_effort': 0.0, 'allow_stalling': True,
        'stall_velocity_threshold': .0001, 'stall_timeout': 1.0}}
    arm = result[side+'_arm_controller']['ros__parameters']
    arm['constraints'].update({f'{side}_j{i}': {'trajectory': .05, 'goal': .01} for i in range(1, 7)})
    return result
