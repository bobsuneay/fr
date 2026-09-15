"""Real hardware adapter; no simulator service or commanded-pose feedback."""
import time
import numpy as np
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from ros2_hkv_gripper.msg import GripperRegisters
from .ros_io import IO, matrix
from .core import transform
from .real_contract import validate_real, fresh, valid_covariance, contact


class RealIO(IO):
    def __init__(self, node):
        super().__init__(node)
        self.real = validate_real(self.c['real'])
        self.fingers = {s: ActionClient(node, GripperCommand, f'/{s}_gripper_controller/gripper_cmd')
                        for s in ('left', 'right')}
        self.registers = {}
        self.observation = None
        self.owner_side = ''
        self.possible_hold = set()
        self.confirmed = set()
        self.monitoring = False
        node.create_subscription(PoseWithCovarianceStamped, self.real['object_pose_topic'],
                                 self.on_pose, qos_profile_sensor_data)
        for side in ('left', 'right'):
            node.create_subscription(GripperRegisters, f'/{side}/gripper_registers',
                lambda msg, side=side: self.on_registers(side, msg), qos_profile_sensor_data)

    def on_pose(self, msg):
        # Require world-frame output so covariance and pose share the same frame.
        try:
            if msg.header.frame_id != 'world' or not valid_covariance(msg.pose.covariance,
                    self.real['max_position_std'], self.real['max_rotation_std']):
                return
            q = msg.pose.pose.orientation
            if abs(np.linalg.norm([q.x, q.y, q.z, q.w])-1) > .001:
                return
            value = matrix(msg.pose.pose)
            if not np.all(np.isfinite(value)):
                return
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            with self.n.data_lock:
                if self.observation is None or stamp > self.observation[1]:
                    self.observation = (value, stamp, time.monotonic())
        except (ValueError, TypeError):
            return

    def on_registers(self, side, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        with self.n.data_lock:
            old = self.registers.get(side)
            if old is None or stamp > old[1]:
                self.registers[side] = (msg, stamp, time.monotonic())

    def require_fresh(self, stamp, received):
        if not fresh(stamp, received, self.n.get_clock().now().nanoseconds*1e-9,
                     time.monotonic(), self.real['feedback_timeout']):
            raise RuntimeError('Real sensor feedback stale; holding state retained')

    def object_pose(self):
        with self.n.data_lock:
            value = self.observation
        if value is None:
            raise RuntimeError('No independent real object pose with valid covariance')
        self.require_fresh(value[1], value[2])
        return value[0].copy()

    def truth(self):
        # Compatibility boundary for the existing task's model-Z verification frame.
        # Incoming real object frame has X along the bolt, including roll.
        return self.object_pose() @ transform(rpy=(0, np.pi/2, 0))

    def finger_position(self, side):
        with self.n.data_lock:
            value = self.n.joints.get(side+'_left_finger_joint')
        if value is None or not 0 <= time.monotonic()-value[1] <= self.real['feedback_timeout']:
            raise RuntimeError('Missing/stale real finger position')
        return value[0]

    def has_contact(self, side):
        with self.n.data_lock:
            value = self.registers.get(side)
        if value is None:
            raise RuntimeError('Missing real HKV registers for '+side)
        self.require_fresh(value[1], value[2])
        msg = value[0]
        return contact(self.real[side], [msg.x1_value, msg.y1_value, msg.z1_value,
                       msg.x2_value, msg.y2_value, msg.z2_value], self.finger_position(side))

    def check(self):
        if self.n.stop_event.is_set():
            raise RuntimeError('Stopped; physical grippers remain unchanged')
        # Never use the inherited simulator object-state checks.
        import rclpy
        if not rclpy.ok():
            raise RuntimeError('ROS shutdown')
        if self.monitoring:
            self.state()
            observed = self.object_pose()
            for side in tuple(self.confirmed):
                if not self.has_contact(side):
                    raise RuntimeError('Bilateral contact lost: '+side)
            if self.watch_center is not None:
                if np.linalg.norm(observed[:3, 3]-self.watch_center) > self.c['center_tolerance']:
                    raise RuntimeError('Independent object tracker detected centre drift')

    def grasp_owner(self):
        if self.owner_side:
            if not self.has_contact(self.owner_side):
                raise RuntimeError('Holding evidence lost; ownership is uncertain')
            return self.owner_side
        if self.possible_hold:
            raise RuntimeError('Closure attempted; ownership uncertain, automatic retry forbidden')
        # Starting after a node restart must not silently open already-closed jaws.
        for side in ('left', 'right'):
            if self.has_contact(side) or abs(self.finger_position(side)-self.real[side]['finger_open']) > self.real['opening_tolerance']:
                raise RuntimeError('Start requires both grippers at measured empty-open position')
        return ''

    def state(self):
        value = super().state()
        with self.n.data_lock:
            for name in value.joint_state.name:
                if not 0 <= time.monotonic()-self.n.joints[name][1] <= self.real['feedback_timeout']:
                    raise RuntimeError('Real joint feedback stale: '+name)
        return value

    def gripper(self, side, width):
        target = float(width)/2
        item = self.real[side]
        if not item['finger_closed'] <= target <= item['finger_open']:
            raise ValueError('Requested gap outside calibrated HKV endpoints')
        closing = abs(width-self.c['close_width']) < 1e-8
        if closing:
            self.possible_hold.add(side)  # latch before send, including timeout/late acceptance
        elif side in self.possible_hold:
            if self.owner_side == side or not self.owner_side or not self.has_contact(self.owner_side):
                raise RuntimeError('Cannot open possible holder without confirmed receiver')
            self.confirmed.discard(side)
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = 0.0  # driver force is commissioned, not action force
        result = self.action(self.fingers[side], goal, self.real['gripper_timeout'])
        if not (result.reached_goal or (closing and result.stalled)):
            raise RuntimeError('Real gripper did not reach or stall at the requested goal')
        if not closing:
            if abs(self.finger_position(side)-target) > self.real['opening_tolerance']:
                raise RuntimeError('Real opening position not reached')
            self.possible_hold.discard(side)

    def assisted_grasp(self, side, close=True):
        if not close:
            raise RuntimeError('Real release must use the verified handover transaction')
        end, stable, last_stamp, samples = time.monotonic()+self.real['gripper_timeout'], None, None, 0
        while time.monotonic() < end:
            self.check()
            self.object_pose()
            good = self.has_contact(side)
            with self.n.data_lock:
                stamp = self.registers[side][1]
            if not good:
                stable, samples = None, 0
            elif stamp != last_stamp:
                last_stamp = stamp
                stable = time.monotonic() if stable is None else stable
                samples += 1
                if samples >= 3 and time.monotonic()-stable >= self.real['contact_seconds']:
                    self.confirmed.add(side)
                    self.owner_side = side
                    return
            self.n.stop_event.wait(.02)
        raise RuntimeError('Sustained bilateral HKV contact not confirmed; retain closed jaws')

    def relocate_object(self, world_object):
        raise RuntimeError('Object relocation is simulation-only')
