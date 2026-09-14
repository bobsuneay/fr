# HKV TG-9801 Gripper ROS 2 Controller

[**中文**](README.md) | [**English**](README_en.md)

基于 ROS 2 和 C++ 开发的 HKV TG-9801 串口夹爪控制器。本包提供了高并发安全的串口通信架构，并支持两种截然不同的工作模式以满足不同的应用场景：**ros2_control 模式（力位模式）** 和 **独立服务模式（自动模式）**。

---

## 1. ros2_control 模式 (力位模式)

该模式直接集成到了 `ros2_control` 框架中，使用自定义的 `HardwareInterface` 提供带有预设力约束的闭环位置控制。如果你正在使用 MoveIt! 或者需要连续的轨迹跟随插补，推荐使用此模式。

### 启动与使用
```bash
ros2 launch ros2_hkv_gripper gripper_control.launch.py \
    serial_port:=/dev/ttyACM0 \
    baud_rate:=1000000 \
    launch_rviz:=false
```

#### 话题 (Topics) 与订阅
- **`/tg9801_gripper_controller/commands`** (`std_msgs/msg/Float64MultiArray`)
  在此下发位置控制指令。范围：`[0.0]` (完全张开) 到 `[0.1]` (完全闭合)。*注意：这里的数值代表两指间的物理间距。*
- **`/joint_states`** (`sensor_msgs/msg/JointState`)
  标准的关节状态反馈，供 MoveIt 或 RViz 使用。
- **`/gripper_registers`** (`ros2_hkv_gripper/msg/GripperRegisters`)
  所有 10 个底层硬件寄存器的实时反馈，最高以 100Hz 轮询发布。

#### 服务 (Services)
- **`/tg9801_activation_controller/reactivate_gripper`** (`std_srvs/srv/Trigger`)
  用于重新校准并激活夹爪的服务。

#### 命令行示例
```bash
# 移动到半开位置 (0.05m 间距)
ros2 topic pub --once /tg9801_gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.05]"

# 完全闭合 (0.1m 表示闭合的动作行程)
ros2 topic pub --once /tg9801_gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1]"

# 完全张开 (0.0m)
ros2 topic pub --once /tg9801_gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]"
```
*(注：使用 `--once` 发送指令时，如果看到 `Waiting for at least 1 matching subscription(s)...`，这是正常的 DDS 发现延迟现象，它在等待控制器节点匹配到发布者。)*

<details>
<summary>Python 客户端示例代码</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/tg9801_gripper_controller/commands',
            10
        )
    
    def move_to(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        self.pub.publish(msg)

def main():
    rclpy.init()
    client = GripperClient()
    client.move_to(0.05) # 半开
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```
</details>

---

## 2. 独立服务模式 (自动模式)

此模式运行一个独立的、轻量级的 ROS 2 节点 (`GripperControllerNode`)，去除了 `ros2_control` 的所有框架开销。它利用夹爪硬件自带的“自动模式”，通过 ROS 2 服务执行开合动作，非常适合简单的、异步的抓取和放置（Pick-and-Place）任务。

### 启动与使用
```bash
# 启动集成了服务和监控的统一节点 (默认开启监控)
ros2 launch ros2_hkv_gripper gripper_launch.py serial_port:=/dev/ttyACM0 baud_rate:=1000000
```

#### 禁用监控轮询
如果你想完全释放串口，阻止任何后台的 Modbus 读取请求，可以通过 `enable_monitor` 启动参数禁用自动监控：
```bash
ros2 launch ros2_hkv_gripper gripper_launch.py serial_port:=/dev/ttyACM0 baud_rate:=1000000 enable_monitor:=false
```

#### 话题 (Topics)
- **`/gripper/registers`** (`ros2_hkv_gripper/msg/GripperRegisters`)
  以 100Hz 发布的寄存器数据（仅在 `enable_monitor:=true` 时有效）。

#### 服务 (Services)
- **`/gripper_command`** (`ros2_hkv_gripper/srv/GripperCommand`)
  控制夹爪张开或闭合。速度参数是可选的（范围 200~1500，默认 1000）。
- **`/read_finger_state`** (`ros2_hkv_gripper/srv/ReadFingerState`)
  手动单次拉取当前状态（在禁用自动监控时非常有用）。

#### 命令行示例
```bash
# 以速度 800 闭合夹爪
ros2 service call /gripper_command ros2_hkv_gripper/srv/GripperCommand "{command: true, speed: 800}"

# 以默认速度张开夹爪
ros2 service call /gripper_command ros2_hkv_gripper/srv/GripperCommand "{command: false}"

# 查看寄存器监控数据
ros2 topic echo /gripper/registers
```

---

## 3. 寄存器映射表

| 索引 | 地址 | 名称 | 取值范围 | 描述 |
|-------|---------|------|-------|------------|
| 0 | 0x0000 | X1 | -32767~32767 |  |
| 1 | 0x0001 | Y1 | -32767~32768 |  |
| 2 | 0x0002 | Z1 | -32767~32769 |  |
| 3 | 0x0003 | X2 | -32767~32770 |  |
| 4 | 0x0004 | Y2 | -32767~32771 |  |
| 5 | 0x0005 | Z2 | -32767~32772 |  |
| 6 | 0x0006 | 状态 | 0/1/2/3 | 0=空闲, 1=夹取中, 2=等待, 3=已夹紧 |
| 7 | 0x0007 | 硬度 | 0~6000 | 抓取物硬度检测 |
| 8 | 0x0008 | 位置 | 0~100 | 实时位置反馈 (0=闭合, 100=张开) |
| 9 | 0x0009 | 电流 | 0~1400 | 电流反馈 |

---

## 4. 安装与配置

### 依赖项
- ROS 2 (Humble 或更高版本)
- `hardware_interface`
- `controller_interface`
- `controller_manager`
- `robot_state_publisher`
- `joint_state_broadcaster`
- `forward_command_controller`
- `xacro`

### 安装步骤
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
# source /opt/ros/jazzy/setup.bash

# 安装 ros2_control 相关的运行/构建依赖
sudo apt-get update
sudo apt-get install -y \
  ros-humble-hardware-interface \
  ros-humble-controller-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro

# 对于 ROS 2 Jazzy (Ubuntu 24.04)，请使用：
# sudo apt-get install -y \
#   ros-jazzy-hardware-interface \
#   ros-jazzy-controller-interface \
#   ros-jazzy-ros2-control \
#   ros-jazzy-ros2-controllers \
#   ros-jazzy-xacro

# 克隆并编译
cd ~/ros2_ws/src
git clone https://gitee.com/hkv233/ros2_hkv_gripper.git
cd ~/ros2_ws
colcon build --packages-select ros2_hkv_gripper
source install/setup.bash
```

### 配置项 (ros2_control)

**硬件参数 (URDF/xacro)**
编辑 `urdf/gripper.urdf.xacro` 文件：
```xml
<hardware>
  <plugin>ros2_hkv_gripper/GripperHardwareInterface</plugin>
  <param name="serial_port">/dev/ttyACM0</param>
  <param name="baud_rate">1000000</param>
  <param name="timeout">1000</param>
  <param name="slave_address">1</param>
  <param name="gripper_model">TG-9801</param>
  <param name="gripper_closed_position">0.1</param>
  <param name="position_mode_speed_register">1000</param>
</hardware>
```

**控制器参数**
编辑 `config/gripper_controllers.yaml` 文件：
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

tg9801_gripper_controller:
  ros__parameters:
    joints:
      - gripper_joint
    interface_name: position
```

### 常见问题排查 (Troubleshooting)

- **Linux 串口权限问题**：
  ```bash
  # 临时赋予读写权限
  sudo chmod 666 /dev/ttyACM0
  
  # 永久解决：将当前用户加入 dialout 用户组（需注销后重新登录生效）
  sudo usermod -a -G dialout $USER
  ```

- **控制器加载失败**：
  ```bash
  ros2 control list_controllers
  ros2 control list_hardware_interfaces
  ros2 run controller_manager ros2_control_node --ros-args --log-level debug
  ```

## 许可证 (License)
AGPL-3.0
