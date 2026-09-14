#!/usr/bin/env python3
"""
Interactive gripper control marker for RViz.
Publishes Float64MultiArray commands to /tg9801_gripper_controller/commands
based on marker feedback (slider position 0.0~1.0 normalized to gripper command).
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64MultiArray
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
)
import math


class RVizGripperCmdMarker(Node):
    def __init__(self):
        super().__init__("rviz_gripper_cmd_marker")
        
        # Parameters
        self.declare_parameter("marker_frame", "base_link")
        self.declare_parameter("marker_name", "gripper_cmd_marker")
        self.declare_parameter("update_rate_hz", 20.0)
        
        self.marker_frame = self.get_parameter("marker_frame").value
        self.marker_name = self.get_parameter("marker_name").value
        self.update_rate = self.get_parameter("update_rate_hz").value
        
        # Publisher for gripper commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/tg9801_gripper_controller/commands",
            10,
        )
        
        # Interactive marker server
        self.server = InteractiveMarkerServer(self, "gripper_cmd_marker_server")
        
        # Current command value (0.0~1.0)
        self.current_command = 0.5
        
        # Timer for publishing at fixed rate
        callback_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_command,
            callback_group=callback_group,
        )
        
        # Create the interactive marker
        self.create_gripper_marker()
        
        self.get_logger().info(
            f"RViz Gripper Control Marker initialized. "
            f"Frame: {self.marker_frame}, "
            f"Update rate: {self.update_rate} Hz"
        )

    def create_gripper_marker(self):
        """Create an interactive marker with a slider for gripper control."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.marker_frame
        int_marker.name = self.marker_name
        int_marker.description = "Gripper Command Slider (0=Closed, 1=Open)"
        
        # Position marker above the gripper base
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.15
        int_marker.pose.orientation.w = 1.0
        
        # Visual marker (red cylinder)
        box_marker = Marker()
        box_marker.type = Marker.CYLINDER
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.2
        box_marker.color.r = 1.0
        box_marker.color.g = 0.2
        box_marker.color.b = 0.2
        box_marker.color.a = 0.8
        
        # Control for movement along Z-axis (slider)
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.z = 1.0
        control.marker = box_marker
        
        int_marker.controls.append(control)
        
        # Add a text control to show current value
        text_control = InteractiveMarkerControl()
        text_control.name = "text_info"
        text_control.interaction_mode = InteractiveMarkerControl.NONE
        text_control.always_visible = True
        
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.scale.x = 0.1
        text_marker.scale.y = 0.1
        text_marker.scale.z = 0.05
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"Command: {self.current_command:.2f}"
        text_marker.pose.position.z = 0.15
        text_control.markers.append(text_marker)
        
        int_marker.controls.append(text_control)
        
        # Add the marker to the server
        self.server.insert(int_marker, feedback_callback=self.marker_feedback)
        self.server.applyChanges()

    def marker_feedback(self, feedback):
        """Handle feedback from the interactive marker."""
        if feedback.event_type == feedback.POSE_UPDATE:
            # Extract Z position (0.0~0.2 range on Z-axis)
            # Map to 0.0~1.0 command range
            z_pos = feedback.pose.position.z
            # Assuming slider moves from 0.1 to 0.2 (0.1 range)
            # Normalize: (z - 0.1) / 0.1 -> clamp to [0, 1]
            normalized_value = max(0.0, min(1.0, (z_pos - 0.1) / 0.1))
            self.current_command = normalized_value
            
            # Update marker position to reflect bounds
            int_marker = self.server.get(self.marker_name)
            if int_marker:
                int_marker.pose.position.z = max(0.1, min(0.2, z_pos))
                # Update text on marker
                for ctrl in int_marker.controls:
                    if ctrl.name == "text_info":
                        for marker in ctrl.markers:
                            if marker.type == Marker.TEXT_VIEW_FACING:
                                marker.text = f"Command: {self.current_command:.2f}"
                
                self.server.insert(int_marker)
                self.server.applyChanges()
            
            self.get_logger().debug(f"Marker feedback: Z={z_pos:.3f}, Command={self.current_command:.3f}")

    def publish_command(self):
        """Publish the current gripper command at fixed rate."""
        msg = Float64MultiArray()
        msg.data = [self.current_command]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RVizGripperCmdMarker()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
