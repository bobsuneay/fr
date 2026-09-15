from pathlib import Path
import sys
from copy import deepcopy
import numpy as np
import pytest
import yaml

SHARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SHARE))
from fr3_bolt_inspection_cell.real_contract import validate_real, fresh, valid_covariance, contact


def measured_config():
    cfg = yaml.safe_load((SHARE/'config/real_feedback.example.yaml').read_text())
    cfg['calibrated'] = True
    for side in ('left', 'right'):
        cfg[side].update(contact_min=10, contact_max=100)
    return cfg


def test_placeholder_cannot_enable_real_hardware():
    cfg = yaml.safe_load((SHARE/'config/real_feedback.example.yaml').read_text())
    with pytest.raises(ValueError):
        validate_real(cfg)
    cfg['calibrated'] = True
    with pytest.raises(ValueError):
        validate_real(cfg)
    assert validate_real(measured_config())


@pytest.mark.parametrize('stamp,received,valid', [(9.8, 19.8, True), (9., 19.9, False),
    (9.9, 19., False), (10.1, 20., False), (float('nan'), 20., False)])
def test_timestamp_and_receipt_must_both_be_fresh(stamp, received, valid):
    assert fresh(stamp, received, 10., 20., .5) is valid


def test_uncertainty_rejects_unknown_non_psd_and_poor_tracking():
    cov = np.diag([1e-6]*3+[1e-4]*3)
    assert valid_covariance(cov, .002, .05)
    assert not valid_covariance(np.zeros((6, 6)), .002, .05)
    bad = cov.copy()
    bad[0, 0] = .1
    assert not valid_covariance(bad, .002, .05)
    bad = cov.copy()
    bad[0, 1] = bad[1, 0] = 1
    assert not valid_covariance(bad, .002, .05)


def test_bilateral_contact_needs_two_fingers_and_plausible_measured_gap():
    cfg = measured_config()['left']
    assert contact(cfg, [20, 0, 0, 20, 0, 0], .006)
    assert not contact(cfg, [20, 0, 0, 0, 0, 0], .006)
    assert not contact(cfg, [20, 0, 0, 20, 0, 0], .001)
    assert not contact(cfg, [200, 0, 0, 20, 0, 0], .006)
    cfg['finger1_zero'] = [20, 0, 0]
    assert not contact(cfg, [20, 0, 0, 20, 0, 0], .006)
