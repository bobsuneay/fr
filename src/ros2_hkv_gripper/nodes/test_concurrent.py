#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_hkv_gripper.msg import GripperRegisters
from ros2_hkv_gripper.srv import GripperCommand
import time
import threading

class GripperTestNode(Node):
    def __init__(self):
        super().__init__('gripper_test_node')
        
        self.subscription = self.create_subscription(
            GripperRegisters,
            '/gripper/registers',
            self.listener_callback,
            10)
        self.msg_count = 0
        self.last_position = -1
        self.last_status = -1
        
        # 用于计算频率和延迟
        self.last_msg_time = 0.0
        self.hz_list = []
        
        self.cli = self.create_client(GripperCommand, '/gripper_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
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
        
        # 只在状态或位置发生显著变化时打印，避免刷屏
        if abs(msg.position - self.last_position) > 10 or msg.status != self.last_status:
            avg_hz = sum(self.hz_list)/len(self.hz_list) if self.hz_list else 0.0
            self.get_logger().info(f'[Monitor] Msg #{self.msg_count} | Status: {msg.status}, Position: {msg.position} | Avg Hz: {avg_hz:.1f}')
            self.last_position = msg.position
            self.last_status = msg.status

    def send_request(self, command, speed):
        req = GripperCommand.Request()
        req.command = command
        req.speed = speed
        self.get_logger().info(f'[Control] Sending command={"Grip" if command else "Release"}, speed={speed}...')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def run_control_sequence(node):
    time.sleep(2) # 等待节点完全启动并收集初始频率数据
    
    for i in range(3):
        node.get_logger().info(f'\n--- Starting Cycle {i+1}/3 ---')
        
        # 夹紧
        result = node.send_request(command=True, speed=800)
        node.get_logger().info(f'[Control] Grip Result: success={result.success}')
        time.sleep(3) # 等待动作完成
        
        # 松开
        result = node.send_request(command=False, speed=800)
        node.get_logger().info(f'[Control] Release Result: success={result.success}')
        time.sleep(3) # 等待动作完成
        
    node.get_logger().info('\n--- All cycles completed. Exiting... ---')

def main(args=None):
    rclpy.init(args=args)
    node = GripperTestNode()
    
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
