#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_hkv_gripper.msg import GripperRegisters
from std_msgs.msg import Float64MultiArray
import time
import threading

class GripperRegistersTestNode(Node):
    def __init__(self):
        super().__init__('gripper_registers_test_node')
        
        # 订阅 /gripper_registers 话题 (包含 10 个寄存器)
        self.subscription = self.create_subscription(
            GripperRegisters,
            '/gripper_registers',
            self.listener_callback,
            10)
        self.msg_count = 0
        self.last_position = -1
        
        # 创建发布者，用于发送位置指令 (forward_command_controller)
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/tg9801_gripper_controller/commands',
            10)
            
    def listener_callback(self, msg):
        self.msg_count += 1
        
        # 只在位置发生显著变化时打印
        if abs(msg.position - self.last_position) > 10:
            self.get_logger().info(f'[Monitor] Msg #{self.msg_count} | Position: {msg.position} | Force Z1: {msg.z1_value} | Status: {msg.status}')
            self.last_position = msg.position

    def send_position_command(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        self.get_logger().info(f'[Control] Sending position command: {position:.3f}m ...')
        self.cmd_pub.publish(msg)

def run_control_sequence(node):
    time.sleep(2) # 等待节点完全启动并收集初始频率数据
    
    for i in range(2):
        node.get_logger().info(f'\n--- Starting Cycle {i+1}/2 ---')
        
        # 闭合 (0.1m)
        node.send_position_command(0.1)
        time.sleep(3) # 等待动作完成
        
        # 张开 (0.0m)
        node.send_position_command(0.0)
        time.sleep(3) # 等待动作完成
        
    node.get_logger().info('\n--- All cycles completed. Exiting... ---')

def main(args=None):
    rclpy.init(args=args)
    node = GripperRegistersTestNode()
    
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
