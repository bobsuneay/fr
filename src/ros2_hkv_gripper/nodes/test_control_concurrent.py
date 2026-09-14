#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import threading

class GripperControlTestNode(Node):
    def __init__(self):
        super().__init__('gripper_control_test_node')
        
        # 订阅 joint_states 话题 (ros2_control 发布)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.msg_count = 0
        self.last_position = -1.0
        
        # 用于计算频率和延迟
        self.last_msg_time = 0.0
        self.hz_list = []
        
        # 创建发布者，用于发送位置指令 (forward_command_controller)
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/tg9801_gripper_controller/commands',
            10)
            
    def listener_callback(self, msg):
        current_time = time.time()
        
        if self.last_msg_time > 0:
            dt = current_time - self.last_msg_time
            hz = 1.0 / dt if dt > 0 else 0
            self.hz_list.append(hz)
            if len(self.hz_list) > 50:
                self.hz_list.pop(0) # 保持最近 50 次的数据用于计算平均值
            
            # 正常 100Hz 间隔是 10ms (0.01s)。如果间隔超过 25ms，说明发生了阻塞丢帧
            if dt > 0.025:
                self.get_logger().warn(f'[Monitor] Blocked! Message delay: {dt*1000:.1f} ms (Instant drop to {hz:.1f} Hz)')
                
        self.last_msg_time = current_time
        self.msg_count += 1
        
        # 提取夹爪关节的位置
        try:
            joint_idx = msg.name.index('gripper_joint')
            current_pos = msg.position[joint_idx]
            
            # 只在位置发生显著变化时打印 (0.01m)
            if abs(current_pos - self.last_position) > 0.001:
                avg_hz = sum(self.hz_list)/len(self.hz_list) if self.hz_list else 0.0
                self.get_logger().info(f'[Monitor] Msg #{self.msg_count} | Position: {current_pos:.4f}m | Avg Hz: {avg_hz:.1f}')
                self.last_position = current_pos
        except ValueError:
            pass # 没找到夹爪关节

    def send_position_command(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        self.get_logger().info(f'[Control] Sending position command: {position:.3f}m ...')
        self.cmd_pub.publish(msg)

def run_control_sequence(node):
    time.sleep(2) # 等待节点完全启动并收集初始频率数据
    
    for i in range(3):
        node.get_logger().info(f'\n--- Starting Cycle {i+1}/3 ---')
        
        # 闭合 (0.1m)
        node.send_position_command(0.1)
        time.sleep(2.0)
        
        # 张开 (0.0m)
        node.send_position_command(0.0)
        time.sleep(2.0) # 等待动作完成
        
    node.get_logger().info('\n--- All cycles completed. Exiting... ---')

def main(args=None):
    rclpy.init(args=args)
    node = GripperControlTestNode()
    
    control_thread = threading.Thread(target=run_control_sequence, args=(node,))
    control_thread.start()
    
    try:
        while control_thread.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
