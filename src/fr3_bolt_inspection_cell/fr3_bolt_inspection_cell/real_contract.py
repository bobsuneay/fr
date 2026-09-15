"""ROS-independent commissioning and evidence checks. SI units throughout."""
import math
import numpy as np


def validate_real(cfg):
    if cfg.get('calibrated') is not True:
        raise ValueError('Real calibration is incomplete: set calibrated only after measurement')
    for key in ('feedback_timeout', 'contact_seconds', 'max_position_std', 'max_rotation_std',
                'opening_tolerance', 'gripper_timeout'):
        if not isinstance(cfg.get(key), (int, float)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('Invalid real.'+key)
    if not isinstance(cfg.get('object_pose_topic'), str) or not cfg['object_pose_topic'].startswith('/'):
        raise ValueError('An absolute object_pose_topic is required')
    for source, target in cfg.get('topic_remappings', {}).items():
        if not isinstance(source, str) or not isinstance(target, str) or not source.startswith('/') or not target.startswith('/'):
            raise ValueError('Camera remappings must use absolute ROS topic names')
    for side in ('left', 'right'):
        item = cfg[side]
        for key in ('finger_open', 'finger_closed', 'contact_min', 'contact_max', 'held_gap_min', 'held_gap_max'):
            if not isinstance(item.get(key), (int, float)) or not math.isfinite(item[key]):
                raise ValueError(f'Invalid real.{side}.{key}')
        if not 0 <= item['finger_closed'] < item['finger_open'] <= .05:
            raise ValueError('Finger endpoints must be measured URDF coordinates, in metres')
        if not 0 < item['contact_min'] < item['contact_max']:
            raise ValueError('Measure bilateral force-register thresholds before execution')
        if not 0 < item['held_gap_min'] < item['held_gap_max'] <= 2*item['finger_open']:
            raise ValueError('Invalid measured held gap interval')
        for key in ('finger1_zero', 'finger2_zero'):
            if np.shape(item[key]) != (3,) or not np.all(np.isfinite(item[key])):
                raise ValueError('Require three measured zero offsets per finger')
    return cfg


def fresh(stamp, received, ros_now, wall_now, timeout):
    return (all(math.isfinite(v) for v in (stamp, received, ros_now, wall_now))
            and 0 <= ros_now-stamp <= timeout and 0 <= wall_now-received <= timeout)


def valid_covariance(covariance, position_std, rotation_std):
    cov = np.asarray(covariance, dtype=float).reshape(6, 6)
    if not np.all(np.isfinite(cov)) or not np.allclose(cov, cov.T, atol=1e-9):
        return False
    if np.min(np.linalg.eigvalsh(cov)) < -1e-12:
        return False
    diagonal = np.diag(cov)
    # Zero uncertainty is not accepted as a substitute for a tracker quality estimate.
    return bool(np.all(diagonal > 0) and np.all(diagonal[:3] <= position_std**2)
                and np.all(diagonal[3:] <= rotation_std**2))


def contact(item, forces, finger_position):
    f = np.asarray(forces, dtype=float)
    if f.shape != (6,) or not np.all(np.isfinite(f)) or not math.isfinite(finger_position):
        return False
    magnitudes = (np.linalg.norm(f[:3]-item['finger1_zero']),
                  np.linalg.norm(f[3:]-item['finger2_zero']))
    return (all(item['contact_min'] <= v <= item['contact_max'] for v in magnitudes)
            and item['held_gap_min'] <= 2*finger_position <= item['held_gap_max'])
