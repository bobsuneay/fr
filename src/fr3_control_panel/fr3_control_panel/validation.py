import math


def validate_config(cfg):
    for name in ('group', 'frame', 'tcp', 'joint_topic', 'move_action', 'fk_service',
                 'scene_service', 'gripper_action', 'gripper_joint'):
        if not isinstance(cfg.get(name), str) or not cfg[name].strip():
            raise ValueError('缺少配置 '+name)
    if len(cfg['joints']) != 6 or len(set(cfg['joints'])) != 6:
        raise ValueError('需要六个不同的机械臂关节名')
    for key in ('velocity', 'acceleration', 'feedback_timeout', 'closed_position'):
        value = cfg[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('无效配置 '+key)
    if cfg['velocity'] > 1 or cfg['acceleration'] > 1:
        raise ValueError('速度/加速度比例必须在 (0,1]')
    for key in ('payload_size', 'payload_offset'):
        if len(cfg[key]) != 3 or any(not math.isfinite(x) for x in cfg[key]):
            raise ValueError('无效配置 '+key)
    if min(cfg['payload_size']) <= 0:
        raise ValueError('物体包围盒尺寸必须为正')
    if not cfg['touch_links'] or not all(isinstance(x, str) for x in cfg['touch_links']):
        raise ValueError('需要物体接触连杆名称')
