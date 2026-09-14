# HKV TG-9801 Gripper ROS 2 Controller

[**中文**](README.md) | [**English**](README_en.md)

ROS 2 C++ package for controlling the HKV TG-9801 gripper via serial port. This package provides robust concurrent serial communication and supports two distinct modes of operation to suit different use cases: **ros2_control Mode** and **Standalone Service Mode**.

---

## 1. ros2_control Mode (Force-Position Control)

This mode integrates directly with the `ros2_control` framework, utilizing a custom `HardwareInterface` to provide closed-loop position control with preset force constraints. It is the recommended mode if you are using MoveIt! or require continuous trajectory following.



### Launch & Usage
```bash
ros2 launch ros2_hkv_gripper gripper_control.launch.py \
    serial_port:=/dev/ttyACM0 \
    baud_rate:=1000000 \
    launch_rviz:=false
```

#### Topics & Subscriptions
- **`/tg9801_gripper_controller/commands`** (`std_msgs/msg/Float64MultiArray`)
  Send position commands here. Range: `[0.0]` (open) to `[0.1]` (closed).
- **`/joint_states`** (`sensor_msgs/msg/JointState`)
  Standard joint state feedback for MoveIt/RViz.
- **`/gripper_registers`** (`ros2_hkv_gripper/msg/GripperRegisters`)
  Real-time feedback of all 10 raw hardware registers, polled at up to 100Hz.

#### Services
- **`/tg9801_activation_controller/reactivate_gripper`** (`std_srvs/srv/Trigger`)
  Service to re-calibrate and activate the gripper.

#### Command Examples
```bash
# Move to half-open position (0.05m)
ros2 topic pub --once /tg9801_gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.05]"

# Fully close (0.1m)
ros2 topic pub --once /tg9801_gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1]"

# Fully open (0.0m)
ros2 topic pub --once /tg9801_gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]"
```
*(Note: When using `--once`, you might see `Waiting for at least 1 matching subscription(s)...`. This is normal DDS discovery latency waiting for the controller node to match the publisher.)*

<details>
<summary>Python Client Example</summary>

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
    client.move_to(0.05) # Half open
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```
</details>

---

## 2. Standalone Service Mode (Auto Mode)

This mode runs a single, lightweight node (`GripperControllerNode`) without the `ros2_control` overhead. It uses the gripper's native "Auto Mode" to perform actions via ROS 2 Services, making it ideal for simple asynchronous pick-and-place tasks.


### Launch & Usage
```bash
# Launch unified service and monitor node (Monitor is enabled by default)
ros2 launch ros2_hkv_gripper gripper_launch.py serial_port:=/dev/ttyACM0 baud_rate:=1000000
```

#### Disabling the Monitor
If you want to free up the serial port and prevent any background Modbus read requests, disable the automatic register monitoring via the `enable_monitor` launch argument:
```bash
ros2 launch ros2_hkv_gripper gripper_launch.py serial_port:=/dev/ttyACM0 baud_rate:=1000000 enable_monitor:=false
```

#### Topics & Subscriptions
- **`/gripper/registers`** (`ros2_hkv_gripper/msg/GripperRegisters`)
  Register data published at 100Hz (only active when `enable_monitor:=true`).

#### Services
- **`/gripper_command`** (`ros2_hkv_gripper/srv/GripperCommand`)
  Command the gripper to open or close. Speed parameter is optional (200~1500, default 1000).
- **`/read_finger_state`** (`ros2_hkv_gripper/srv/ReadFingerState`)
  Manually poll the current state (useful if monitor is disabled).

#### Command Examples
```bash
# Close gripper with speed 800
ros2 service call /gripper_command ros2_hkv_gripper/srv/GripperCommand "{command: true, speed: 800}"

# Open gripper with default speed
ros2 service call /gripper_command ros2_hkv_gripper/srv/GripperCommand "{command: false}"

# View register data
ros2 topic echo /gripper/registers
```

---

## 3. Register Mapping

| Index | Address | Name | Range | Description |
|-------|---------|------|-------|------------|
| 0 | 0x0000 | X1 | -32767~32767 |  |
| 1 | 0x0001 | Y1 | -32767~32768 |  |
| 2 | 0x0002 | Z1 | -32767~32769 |  |
| 3 | 0x0003 | X2 | -32767~32770 |  |
| 4 | 0x0004 | Y2 | -32767~32771 |  |
| 5 | 0x0005 | Z2 | -32767~32772 |  |
| 6 | 0x0006 | Status | 0/1/2/3 | 0=idle, 1=gripping, 2=waiting, 3=held |
| 7 | 0x0007 | Hardness | 0~6000 | Object hardness detection |
| 8 | 0x0008 | Position | 0~100 | Real-time position feedback |
| 9 | 0x0009 | Current | 0~1400 | Current feedback |

---

## 4. Installation & Configuration

### Dependencies
- ROS2 (Humble or later)
- `hardware_interface`
- `controller_interface`
- `controller_manager`
- `robot_state_publisher`
- `joint_state_broadcaster`
- `forward_command_controller`

### Installation
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
# source /opt/ros/jazzy/setup.bash

# Install ros2_control runtime/build dependencies
sudo apt-get update
sudo apt-get install -y \
  ros-humble-hardware-interface \
  ros-humble-controller-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro

# For ROS 2 Jazzy (Ubuntu 24.04), use:
# sudo apt-get install -y \
#   ros-jazzy-hardware-interface \
#   ros-jazzy-controller-interface \
#   ros-jazzy-ros2-control \
#   ros-jazzy-ros2-controllers \
#   ros-jazzy-xacro

# Clone and build
cd ~/ros2_ws/src
git clone https://gitee.com/hkv233/ros2_hkv_gripper.git
cd ~/ros2_ws
colcon build --packages-select ros2_hkv_gripper
source install/setup.bash
```

### Configuration (ros2_control)

**Hardware Parameters (URDF/xacro)**
Edit `urdf/gripper.urdf.xacro`:
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

**Controller Parameters**
Edit `config/gripper_controllers.yaml`:
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

### Troubleshooting

- **Serial Port Permissions (Linux)**:
  ```bash
  # Temporary fix
  sudo chmod 666 /dev/ttyACM0
  
  # Permanent fix
  sudo usermod -a -G dialout $USER
  ```

- **Controllers Not Loading**:
  ```bash
  ros2 control list_controllers
  ros2 control list_hardware_interfaces
  ros2 run controller_manager ros2_control_node --ros-args --log-level debug
  ```

## License
AGPL-3.0
