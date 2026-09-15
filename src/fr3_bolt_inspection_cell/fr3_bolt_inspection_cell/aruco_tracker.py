"""Optional, independent camera marker pose for commissioning a marked rigid test part."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from .core import transform
from .ros_io import matrix, pose


class Tracker(Node):
    def __init__(self):
        super().__init__('aruco_object_tracker')
        for name, default in [('image_topic', '/waist_camera/image_raw'),
                              ('info_topic', '/waist_camera/camera_info'),
                              ('output_topic', '/inspection/real/object_pose'),
                              ('marker_id', 0), ('marker_size', 0.03),
                              ('marker_to_object_xyz', [0.0, 0.0, 0.0]),
                              ('marker_to_object_rpy', [0.0, 0.0, 0.0]),
                              ('calibrated', False), ('max_reprojection_px', 1.0),
                              ('position_std', 0.002), ('rotation_std', 0.04)]:
            self.declare_parameter(name, default)
        val = lambda n: self.get_parameter(n).value
        if not val('calibrated'):
            raise ValueError('Measure marker size/extrinsic/object transform and tracker uncertainty first')
        self.marker_size = float(val('marker_size'))
        self.position_std, self.rotation_std = float(val('position_std')), float(val('rotation_std'))
        self.reprojection = float(val('max_reprojection_px'))
        if not all(np.isfinite(v) and v > 0 for v in (self.marker_size, self.position_std, self.rotation_std, self.reprojection)):
            raise ValueError('Invalid marker scale or uncertainty')
        self.marker_object = transform(val('marker_to_object_xyz'), val('marker_to_object_rpy'))
        self.marker_id = val('marker_id')
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.bridge, self.info = CvBridge(), None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, val('output_topic'), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, val('info_topic'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, val('image_topic'), self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        info = self.info
        if info is None or info.header.frame_id != msg.header.frame_id or info.width != msg.width or info.height != msg.height:
            return
        try:
            stamp = Time.from_msg(msg.header.stamp)
            if not 0 <= (self.get_clock().now()-stamp).nanoseconds*1e-9 <= .5:
                return
            gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
            if ids is None or np.count_nonzero(ids == self.marker_id) != 1:
                return
            pixels = corners[int(np.flatnonzero(ids.ravel() == self.marker_id)[0])].reshape(4, 2)
            a = self.marker_size/2
            points = np.array([[-a, a, 0], [a, a, 0], [a, -a, 0], [-a, -a, 0]], np.float64)
            k = np.asarray(info.k, dtype=float).reshape(3, 3)
            d = np.asarray(info.d, dtype=float)
            ok, rvec, tvec = cv2.solvePnP(points, pixels, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or tvec[2, 0] <= 0:
                return
            projected, _ = cv2.projectPoints(points, rvec, tvec, k, d)
            if np.sqrt(np.mean((projected.reshape(4, 2)-pixels)**2)) > self.reprojection:
                return
            camera_marker = np.eye(4)
            camera_marker[:3, :3] = cv2.Rodrigues(rvec)[0]
            camera_marker[:3, 3] = tvec.ravel()
            tr = self.tf.lookup_transform('world', msg.header.frame_id, stamp).transform
            from geometry_msgs.msg import Pose
            world_camera = Pose()
            world_camera.position.x, world_camera.position.y, world_camera.position.z = tr.translation.x, tr.translation.y, tr.translation.z
            world_camera.orientation = tr.rotation
            out = PoseWithCovarianceStamped()
            out.header.stamp, out.header.frame_id = msg.header.stamp, 'world'
            out.pose.pose = pose(matrix(world_camera)@camera_marker@self.marker_object)
            # Empirically calibrated world-frame uncertainty bounds, not a claim of PnP covariance.
            out.pose.covariance = np.diag([self.position_std**2]*3+[self.rotation_std**2]*3).ravel().tolist()
            self.pub.publish(out)
        except Exception as exc:
            self.get_logger().debug('Marker observation rejected: '+str(exc))


def main():
    rclpy.init()
    node = Tracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
