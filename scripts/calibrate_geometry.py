"""Fit recorded calibration samples offline; never moves or enables a robot."""
import argparse
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/fr3_bolt_inspection_cell'))
from fr3_bolt_inspection_cell.calibration import rigid_fit, pivot_fit, require_fit_quality


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=['registration', 'pivot'])
    parser.add_argument('samples', type=Path)
    parser.add_argument('--max-error', type=float, required=True, help='Maximum fit residual in metres')
    parser.add_argument('--output', type=Path, required=True, help='New report file; never overwritten')
    args = parser.parse_args()
    data = yaml.safe_load(args.samples.read_text(encoding='utf-8'))
    if data.get('units') != 'metres':
        parser.error('Input must explicitly declare units: metres')
    if args.kind == 'registration':
        report = rigid_fit(data['source_points'], data['target_points'])
    else:
        report = pivot_fit(data['world_from_tool'])
    report = require_fit_quality(report, args.max_error)
    report.update(kind=args.kind, units='metres', source_samples=str(args.samples),
                  note='Fit residual only; validate independently before copying into deployment config.')
    with args.output.open('x', encoding='utf-8') as output:
        yaml.safe_dump(report, output, sort_keys=False)
    print(yaml.safe_dump(report, sort_keys=False))


if __name__ == '__main__':
    main()
