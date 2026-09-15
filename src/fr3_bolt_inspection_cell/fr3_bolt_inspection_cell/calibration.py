"""Offline rigid registration and fixed-tip pivot calibration, metres/radians."""
import numpy as np
from scipy.spatial.transform import Rotation


def rigid_fit(source, target):
    """Corresponding surveyed points: p_target = R @ p_source + t."""
    a, b = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or a.shape != b.shape or len(a) < 4:
        raise ValueError('Require at least four paired 3D points')
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError('Nonfinite survey sample')
    ac, bc = a-a.mean(axis=0), b-b.mean(axis=0)
    for centered in (ac, bc):
        singular = np.linalg.svd(centered, compute_uv=False)
        if singular[0] < .001 or singular[1] < singular[0]*.001:
            raise ValueError('Survey points are too small or nearly collinear')
    u, _, vt = np.linalg.svd(ac.T @ bc)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    translation = b.mean(axis=0)-rotation @ a.mean(axis=0)
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation
    errors = np.linalg.norm((rotation @ a.T).T+translation-b, axis=1)
    return dict(T_target_source=matrix.tolist(), xyz=translation.tolist(),
                rpy=Rotation.from_matrix(rotation).as_euler('xyz').tolist(), **residuals(errors))


def pivot_fit(world_from_tool):
    """Same stationary tip, varying actual tool poses: R_i*tip+t_i=world_pivot."""
    poses = np.asarray(world_from_tool, dtype=float)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4) or len(poses) < 4:
        raise ValueError('Require four or more world_from_tool 4x4 transforms')
    if not np.all(np.isfinite(poses)):
        raise ValueError('Nonfinite tool pose')
    for pose in poses:
        r = pose[:3, :3]
        if (not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8) or
                not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or
                not np.isclose(np.linalg.det(r), 1, atol=1e-6)):
            raise ValueError('Tool poses must be proper rigid transforms')
    design = np.vstack([np.column_stack((p[:3, :3], -np.eye(3))) for p in poses])
    rhs = -poses[:, :3, 3].reshape(-1)
    solution, _, rank, singular = np.linalg.lstsq(design, rhs, rcond=None)
    if rank < 6 or singular[-1] < singular[0]*.001:
        raise ValueError('Insufficient orientation diversity for pivot calibration')
    errors = np.linalg.norm((design @ solution-rhs).reshape(-1, 3), axis=1)
    return dict(tip_in_tool=solution[:3].tolist(), world_pivot=solution[3:].tolist(),
                condition_number=float(singular[0]/singular[-1]), **residuals(errors))


def residuals(errors):
    return dict(rms_error_m=float(np.sqrt(np.mean(errors**2))), max_error_m=float(np.max(errors)),
                samples=len(errors), independently_validated=False)


def require_fit_quality(report, max_error):
    if not np.isfinite(max_error) or not 0 < max_error <= .01:
        raise ValueError('Choose an explicit residual bound in (0, 0.01] metres')
    if report['max_error_m'] > max_error:
        raise ValueError('Fit residual exceeds requested bound; do not deploy this calibration')
    return report
