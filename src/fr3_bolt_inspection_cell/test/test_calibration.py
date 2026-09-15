import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from fr3_bolt_inspection_cell.calibration import rigid_fit, pivot_fit, require_fit_quality


def test_registration_preserves_frame_direction():
    a = np.array([[0, 0, 0], [.1, 0, 0], [0, .2, 0], [0, 0, .3]])
    r = Rotation.from_euler('xyz', [.4, -.2, .6]).as_matrix()
    t = np.array([.3, -.1, .9])
    report = rigid_fit(a, (r @ a.T).T+t)
    result = np.array(report['T_target_source'])
    np.testing.assert_allclose(result[:3, :3], r, atol=1e-12)
    np.testing.assert_allclose(result[:3, 3], t, atol=1e-12)
    assert require_fit_quality(report, .001)['independently_validated'] is False


def test_registration_rejects_collinear_and_nonfinite_samples():
    a = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])
    with pytest.raises(ValueError):
        rigid_fit(a, a)
    a = a.astype(float)
    a[1, 0] = np.nan
    with pytest.raises(ValueError):
        rigid_fit(a, a)


def test_pivot_recovers_tip_not_inverse_transform():
    tip, world = np.array([.02, -.01, .15]), np.array([.3, .2, .7])
    poses = []
    for r in Rotation.from_euler('xyz', [[0, 0, 0], [.5, 0, 0], [0, -.6, 0], [.3, .4, .2]]).as_matrix():
        p = np.eye(4)
        p[:3, :3], p[:3, 3] = r, world-r @ tip
        poses.append(p)
    report = pivot_fit(poses)
    np.testing.assert_allclose(report['tip_in_tool'], tip, atol=1e-12)
    np.testing.assert_allclose(report['world_pivot'], world, atol=1e-12)


def test_pivot_rejects_degenerate_and_nonrigid_samples():
    poses = np.repeat(np.eye(4)[None], 4, axis=0)
    with pytest.raises(ValueError):
        pivot_fit(poses)
    poses[0, 0, 0] = 2
    with pytest.raises(ValueError):
        pivot_fit(poses)


def test_large_residual_is_not_accepted_as_commissioned():
    with pytest.raises(ValueError):
        require_fit_quality({'max_error_m': .005}, .001)
    with pytest.raises(ValueError):
        require_fit_quality({'max_error_m': 0}, float('nan'))
