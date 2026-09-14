"""ROS/Qt independent teaching data, SI conversions and validation."""
import json
import math
from pathlib import Path


def pose_values(values):
    if len(values) != 6 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                               or not math.isfinite(x) for x in values):
        raise ValueError('位姿必须包含六个有限数值 [x,y,z,rx,ry,rz]，单位 m/deg')
    return list(map(float, values))


def opening_to_joint(percent, closed):
    if not math.isfinite(percent) or not 0 <= percent <= 100 or closed <= 0:
        raise ValueError('开度必须在 0–100%')
    return closed * (1 - percent / 100)


def save_points(path, frame, tcp, points):
    data = {'version': 1, 'frame': frame, 'tcp': tcp,
            'points': {k: pose_values(v) for k, v in points.items()}}
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    temp.replace(path)


def load_points(path, frame, tcp):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('version') != 1 or data.get('frame') != frame or data.get('tcp') != tcp:
        raise ValueError('点位文件版本、参考坐标系或 TCP 不匹配')
    if not isinstance(data.get('points'), dict):
        raise ValueError('无效点位文件')
    return {k: pose_values(v) for k, v in data['points'].items()}
