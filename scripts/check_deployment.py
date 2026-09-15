"""Offline validation of a deployment directory; never connects to hardware."""
import argparse
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/fr3_bolt_inspection_cell'), str(ROOT/'src/fr3_dual_bolt_cell')]
from fr3_dual_bolt_cell.model import build_model, validate_hardware, validate_arms
from fr3_bolt_inspection_cell.core import validate
from fr3_bolt_inspection_cell.model import augment
from fr3_bolt_inspection_cell.real_model import configure_real_model
from fr3_bolt_inspection_cell.real_contract import validate_real
from fr3_bolt_inspection_cell.geometry import fingertip_geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    read = lambda name: yaml.safe_load((args.directory/(name+'.yaml')).read_text())
    cfg = validate(read('inspection'))
    cfg['real'] = validate_real(read('real_feedback'))
    hardware = validate_hardware(read('hardware'))
    arms = validate_arms(read('arms'))
    scene = read('scene')
    if abs(scene['table']['top_z']-cfg['table_z']) > 1e-6:
        raise ValueError('Measured table height differs between scene and inspection config')
    share = ROOT/'src/fr3_dual_bolt_cell'
    root = configure_real_model(augment(build_model(share, args.directory/'scene.yaml', arms,
        'real', hardware=hardware), cfg, False), cfg)
    for side in ('left', 'right'):
        fingertip_geometry(root, side, lambda uri: share/uri.split('fr3_dual_bolt_cell/')[1])
        if abs(cfg['open_width']/2-cfg['real'][side]['finger_open']) > cfg['real']['opening_tolerance']:
            raise ValueError('Open width differs from calibrated endpoint: '+side)
    print('Configuration and model valid: 12 robot axes, 2 gripper command joints, 0 Gazebo plugins.')
    print('Files checked only; no calibration accuracy certification or hardware connection.')


if __name__ == '__main__':
    main()
