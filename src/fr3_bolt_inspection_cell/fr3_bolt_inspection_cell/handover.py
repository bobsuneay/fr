"""Small testable transaction: receiver verification precedes donor opening."""
import numpy as np
from copy import deepcopy
from .core import transform
from .cartesian import CartesianPlanningError


def approach_receiver(io, side, object_pose, receiver_grasp, distance, speed, log,
                      alternative_offsets=(), before_execute=None, plan_only=False,
                      start_state=None):
    """Preflight perpendicular approaches from both sides and two wrist rolls."""
    from .ros_io import PlanningFailure
    if start_state is not None and not plan_only:
        raise ValueError('Hypothetical receiver state is for planning only')
    failures = []
    selected = None
    offsets = [float(receiver_grasp[0, 3])]
    for value in alternative_offsets:
        if not np.isfinite(value):
            raise ValueError('Receiver offset must be finite')
        if all(abs(value-prior) > 1e-6 for prior in offsets):
            offsets.append(float(value))
    candidates = [(offset, a, b) for offset in offsets
                  for a, b in ((0, 0), (np.pi, 0), (0, np.pi), (np.pi, np.pi))]
    for offset, side_turn, wrist_turn in candidates:
        grasp = receiver_grasp.copy()
        grasp[0, 3] = offset
        grasp[:3, :3] = (transform(rpy=(side_turn, 0, 0))[:3, :3]@
                         receiver_grasp[:3, :3]@transform(rpy=(0, 0, wrist_turn))[:3, :3])
        target = object_pose@grasp
        pre = target.copy()
        pre[:3, 3] -= target[:3, 2]*distance
        log(f'Perpendicular handover preflight: axial_offset={offset*1000:.2f} mm, side_turn={np.degrees(side_turn):.0f}, '
            f'wrist_turn={np.degrees(wrist_turn):.0f}, pre={np.round(pre[:3, 3], 4).tolist()}')
        try:
            connection, prepared = io.pick_approach(side, pre, target, plan_only=True,
                **({'start_state': start_state} if start_state is not None else {}))
        except PlanningFailure as exc:
            failures.append(str(exc))
            log(f'Perpendicular handover candidate rejected: {exc}')
            continue
        selected = connection, prepared, grasp, target
        break
    if selected is None:
        raise PlanningFailure(f'All {len(candidates)} perpendicular handover candidates failed; no receiver motion: '+' | '.join(failures))
    connection, prepared, grasp, target = selected
    if plan_only:
        return connection, prepared, grasp, target
    log(f'Perpendicular handover selected: axial_offset={grasp[0, 3]*1000:.2f} mm; full approach validated')
    # Execution errors must never cause automatic exploration of another pose.
    if before_execute is not None:
        before_execute()
    io.execute(connection)
    io.execute_prepared_cartesian(prepared, speed)
    return grasp, target


def donor_end_state(start, trajectory, side):
    """Move only donor joints in a private, hypothetical full-robot state."""
    from .ros_io import PlanningFailure
    joint_path = trajectory.joint_trajectory
    names = list(joint_path.joint_names)
    expected = {f'{side}_j{i}' for i in range(1, 7)}
    if len(names) != 6 or set(names) != expected or not joint_path.points:
        raise PlanningFailure('Invalid donor trajectory joints or empty path')
    values = joint_path.points[-1].positions
    if len(values) != 6 or not np.all(np.isfinite(values)):
        raise PlanningFailure('Invalid donor trajectory endpoint')
    result = deepcopy(start)
    positions = list(result.joint_state.position)
    for name, value in zip(names, values):
        positions[result.joint_state.name.index(name)] = float(value)
    result.joint_state.position = positions
    return result


def prepare_cooperative_handover(io, first, second, nominal, donor_grasp, receiver_grasp,
                                cfg, settle, verify, log):
    """Preflight both arms before moving donor; receiver replans after arrival."""
    from .ros_io import PlanningFailure
    start = io.state()
    failures = []
    selected = None
    alternatives = [(cfg['bolt_length']-cfg['head_length'])/2]
    receiver_links = [second+'_left_finger', second+'_right_finger']
    for index, candidate in enumerate(cfg['handover_donor_candidates'], 1):
        io.check()
        target = nominal.copy()
        # Roll about the estimated part's longitudinal X, not joint six.
        target[:3, :3] = nominal[:3, :3]@transform(
            rpy=(np.radians(candidate['roll_deg']), 0, 0))[:3, :3]
        offset = np.asarray(candidate['offset'], dtype=float).copy()
        if second == 'right':
            offset[1] *= -1
        target[:3, 3] += offset
        log(f'Cooperative handover {index}/{len(cfg["handover_donor_candidates"])}: '
            f'donor_roll={candidate["roll_deg"]:.0f} deg, '
            f'world_offset={np.round(offset, 4).tolist()}; preflight only')
        try:
            io.allow_touch(receiver_links, False)
            path = io.global_move(first, target@donor_grasp, plan_only=True, start_state=start)
            end = donor_end_state(start, path, first)
            io.validate_robot_state(end)
            # Use the actual planned FK endpoint, including pose-goal tolerances.
            object_pose = io.tcp_pose(first, robot_state=end)@np.linalg.inv(donor_grasp)
            io.allow_touch(receiver_links)
            try:
                approach_receiver(io, second, object_pose, receiver_grasp,
                    cfg['approach_height'], cfg['descent_speed'], log,
                    alternative_offsets=alternatives, plan_only=True, start_state=end)
            finally:
                io.allow_touch(receiver_links, False)
        except (PlanningFailure, CartesianPlanningError) as exc:
            failures.append(f'candidate {index}: {exc}')
            log(f'Cooperative handover candidate rejected: {exc}')
            continue
        selected = path, object_pose
        break
    if selected is None:
        raise PlanningFailure('No jointly feasible handover; neither arm moved: '+' | '.join(failures))
    path, object_pose = selected
    io.check()
    if io.grasp_owner() != first:
        raise RuntimeError('Donor ownership changed during preflight; no handover motion')
    log('Cooperative handover selected: both paths preflighted; moving donor first')
    # Execution failures propagate; never explore another pose after a failed move.
    io.execute(path)
    settle()
    verify(object_pose)
    return object_pose


def transfer(io, first, second, object_pose, close_width, open_width, settle, verify):
    io.gripper(second, close_width)
    io.assisted_grasp(second)
    io.object_scene(object_pose, second, previous=first)
    settle()
    verify(object_pose)
    if io.grasp_owner() != second:
        raise RuntimeError('Receiver ownership not confirmed; donor stays closed')
    io.gripper(first, open_width)
