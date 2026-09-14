# FR3 + HKV PyQt 控制台

面向 Ubuntu 22.04 / ROS 2 Humble / PyQt5。默认 FR3，基于用户提供的
`frcobot_ros2-v3.0.0_robotV3.9.7` 与 `ros2_hkv_gripper`，复用随本交付提供的
`fr3_real_bringup` 单臂组合模型。Windows 可运行 `--demo` 查看界面，不能加载 Linux SDK。

## 功能

- 六个关节实时角度（deg），夹爪开度百分比，TCP 位姿（m / deg）。
- 显式“连接机械臂与夹爪/断开连接”状态管理：连接按钮会检查 MoveIt、TCP FK、
  PlanningScene 和夹爪 action，同时确认关节反馈有效；未连接时所有运动和采集控制禁用。
- TCP 来自 MoveIt `/compute_fk` 与当前关节反馈，参考系默认为 `base_link`。
  Rx/Ry/Rz 为固定轴 XYZ 欧拉角，不读取控制柜另行配置的工具坐标。
- 当前 TCP 填入目标框、手动采集抓取/展示/途经点、双击编辑、JSON 保存载入。
- 仅规划并向 RViz 发布轨迹，或重新规划并执行至指定笛卡尔位姿。
- 抓取流程：张开 → 规划至抓取点 → 闭合 → 检查反馈 → 挂载物体包围盒
  → 按采集顺序到途经点 → 到展示点，结束时保持夹持。
- 盒形障碍物使用目标框的中心位姿与长宽高加入 MoveIt 场景，同名更新。
- 软件取消、反馈过期拒绝执行、动作失败停止流程、停止状态不明时锁定新任务。

界面不切换机械臂拖动示教模式。可用控制柜/示教器的已有手动功能摆到关键位置，
静止后点击采集；执行前退出手动模式，避免同时由两处控制。

## 1. 独立工作空间构建与 mock

构建后可以直接使用一键脚本（Ubuntu）：

```bash
bash ~/fr3_panel_ws/install/fr3_control_panel/share/fr3_control_panel/scripts/start_panel.sh
```

如果脚本无法自动定位工作空间，可显式指定：

```bash
FR3_PANEL_WS=~/fr3_panel_ws bash ~/fr3_panel_ws/install/fr3_control_panel/share/fr3_control_panel/scripts/start_panel.sh
```

它默认启动虚拟机械臂、MoveIt、RViz 和 PyQt 界面。脚本也支持：

```bash
bash .../start_panel.sh --ui-only  # 只看界面，不连接 ROS
bash .../start_panel.sh --real     # 使用已完成验收的 real.yaml 启动真机
```

真实模式可以用环境变量指定配置：`FR3_REAL_CONFIG=/path/real.yaml`、
`FR3_CELL_CONFIG=/path/cell.yaml`、`HKV_SERIAL_PORT=/dev/ttyACM0`、
`HKV_BAUD_RATE=1000000`。真实模式脚本会传入 `enable_execution:=true`，
只能在现场安全检查完成后使用。Windows 双击 `scripts/start_panel.bat` 只能打开离线 UI 预览。

建议只把本交付的两个包复制到干净工作空间，避免现有双臂工程中的同名包/节点冲突。
假设本交付解压到 `~/fr3_panel_delivery`：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-scipy python3-yaml \
  python3-colcon-common-extensions ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
mkdir -p ~/fr3_panel_ws/src
cp -a ~/fr3_panel_delivery/src/fr3_control_panel ~/fr3_panel_ws/src/
cp -a ~/fr3_panel_delivery/src/fr3_real_bringup ~/fr3_panel_ws/src/
cd ~/fr3_panel_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch fr3_real_bringup mock.launch.py
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/fr3_panel_ws/install/setup.bash
ros2 run fr3_control_panel panel --mock
```

`--mock` 仅配合虚拟 bringup，允许虚拟夹爪空闭合后继续演示；它不会把真实驱动变成虚拟驱动。
界面打开后先点击“连接机械臂与夹爪”。连接检查通过后状态显示“已连接 / 就绪”，
运动、夹爪、位姿采集和抓取流程按钮才会启用；点击“断开连接”会取消当前动作并锁定这些按钮。
连接检查失败时先查看 MoveIt、`/compute_fk`、`/apply_planning_scene` 和
`/tg9801_gripper_controller/gripper_cmd` 是否已启动。
也可以用 `ros2 launch fr3_control_panel mock_panel.launch.py` 一次启动模型、MoveIt、RViz 和面板
（不要和上面的两个启动命令重复运行）。
RViz 的 MotionPlanning 面板可以先移动虚拟臂，再回本界面采集关键点。
若看不到“仅规划”轨迹，在 RViz 添加 MotionPlanning/Trajectory 显示并选择
`/display_planned_path`，或用其默认规划轨迹话题 `/display_planned_path`。

仅查看界面（不需要 ROS）：

```bash
cd src/fr3_control_panel
python -m fr3_control_panel.app --demo
```

需要安装 PyQt5 和 PyYAML；离线预览禁用设备操作与采集，显示“无设备连接”。

## 2. 连接用户提供的硬件包

控制柜确为 3.9.7 时，把厂商仓库内 `fairino_msgs`、`fairino_hardware_v3_9_7`
以及 `ros2_hkv_gripper` 复制到独立 `~/fr3_driver_ws/src`，安装依赖并编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/fr3_driver_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to fairino_hardware_v3_9_7 ros2_hkv_gripper
source install/setup.bash
```

不要同时加载其他版本的同名法奥硬件插件。不必再启动厂商独立 MoveIt demo 或
夹爪 `gripper_control.launch.py`：`fr3_real_bringup` 已在一个 controller_manager
中统一加载两者，额外启动会争用串口或控制器。

随后按 `fr3_real_bringup/docs/COMMISSIONING.md` 配置真实控制柜 IP、版本及
`real.yaml`。`fr3_real_bringup/config/cell.yaml` 中法兰/TCP 偏移是占位值，
必须替换为实测值；这里的 TCP 标定直接影响采集与运动目标。
夹爪力、串口波特率和控制周期由 bringup/硬件参数设置，界面不覆盖这些参数。

启动已完成现场配置的真实系统：

```bash
source /opt/ros/humble/setup.bash
source ~/fr3_driver_ws/install/setup.bash
source ~/fr3_panel_ws/install/setup.bash
ros2 launch fr3_real_bringup bringup.launch.py \
  mode:=real confirm_real:=true enable_execution:=true \
  real_config:=$HOME/fr3_config/real.yaml \
  cell:=$HOME/fr3_config/cell.yaml \
  serial_port:=/dev/ttyACM0 baud_rate:=1000000
```

另一个同样 source 环境的终端运行 `ros2 run fr3_control_panel panel`，不要加 `--mock`。
界面打开后仍需点击“连接机械臂与夹爪”；连接通过不代表硬件使能或急停状态已验证。
现有官方驱动启动后可能持续下发 ServoJ；面板取消不是硬件急停，也不负责控制柜使能。

## 3. 碰撞检测的范围与配置

运动通过 MoveIt `MoveGroup` action 发起，OMPL 对组合 URDF 中的机械臂、工具、桌子
以及 PlanningScene 障碍物做自碰撞/环境碰撞检查。不是直接 SDK MoveL/MoveJ。
“规划并到达”使用当时最新场景重新规划，不复用之前预览的旧轨迹。
目标是笛卡尔位姿，但轨迹可以绕行，不保证 TCP 沿直线运动。
OMPL 使用离散碰撞检测；当前组合配置 `longest_valid_segment_fraction: 0.002`，
它不提供连续碰撞或动态障碍物安全保证。未建模的真实障碍物不会被自动发现。

编辑面板配置副本后使用：

```bash
ros2 run fr3_control_panel panel --config /absolute/path/panel.yaml
```

`config/panel.yaml` 中：

- `frame`、`tcp`、`group`、关节/话题/action 名可配置。更换机械臂型号还需要匹配的
  URDF、SRDF、运动学和控制器；仅修改面板组名不足以切换实体型号。
- `velocity` / `acceleration` 默认 0.1，为 MoveIt 最大值的比例。
- `payload_size` 与 `payload_offset` 是抓取物包围盒及其在 TCP 下的位置，单位 m。
  默认 40×40×80 mm 仅示例，必须按物体改。盒的方向与 TCP 一致。
- `touch_links` 只允许物体与这些工具连杆接触，不豁免与手臂/桌子的碰撞。
- `closed_position: 0.1` 为夹爪闭合关节坐标；界面 100% 对应张开坐标 0。
  百分比是按驱动行程归一化，不代表经过测量的实际毫米间距。

夹取后将物体作为 attached collision object 参与搬运规划；释放时从工具分离，
MoveIt 将其保留在世界场景。当前流程用于一次抓取/展示，不包含自动识别、再次抓取已释放物体
及物体库存管理。抓取前的物体接触区由人工示教，目标物不是普通障碍盒；
不要把同一待抓物作为阻挡夹爪的普通障碍物加入后又期待规划穿入。

真实模式使用夹爪动作的 `stalled` 作为接触依据，完全闭合且没有停滞会报可能空抓，
不进入搬运。停滞也可能来自机械卡阻，不等于可靠识别到物体；实际夹取判定需要现场验证。
若位置模式驱动不会报告停滞，需接入已验证的寄存器/力反馈，不能靠放宽空抓判断处理。

## 4. 使用步骤

1. 等待六轴、夹爪开度和 TCP 出现有效反馈，核对模型和设备一致。
2. 手动摆至抓取位置，静止后点击“采集抓取位姿”。
3. 同理采集“展示位姿”；需要绕行/抬起时依次采集“途经点”。
4. 保存 JSON。文件同时保存参考系和 TCP，坐标系不匹配时拒绝载入。
5. 双击点位，可先“仅规划 / RViz 预览”；检查路径和障碍物。
6. “执行抓取 → 途经点 → 展示”会逐段重新规划，到展示点后保持夹持。
7. 在合适放置位置点击“释放物体”。任何阶段失败都不会自动跳过继续下一段。

抓取前到达动作目前是一段自由空间规划。如果需要严格垂直下探/撤离，应另加
带碰撞检查的 CartesianPath 阶段，不应把该功能当成已经实现的直线插补。

## 5. 验证记录与现场验收

本地 Windows 已通过 Python 编译检查、10 项 pytest（含 Qt 离屏测试）。
另有 ROS 生成消息序列化测试模块因本机未安装 rclpy 跳过；在 ROS 环境中会执行。
测试覆盖开度方向、非法位姿、点位文件参考系、配置限幅、UI 采集、顺序执行，以及
规划失败/空抓后不继续搬运。没有在本机运行 ROS 2、MoveIt 或真实机器人。

```bash
PYTHONPATH=. QT_QPA_PLATFORM=offscreen python -m pytest test -q
```

Ubuntu 上还需验证：mock 采集后规划执行；障碍盒阻挡目标时规划失败；
执行时取消；关闭反馈节点后拒绝新命令；夹取失败不搬运；负载包围盒参与搬运碰撞检测。
这些属于尚未执行的集成验收，不应把本地单元测试结果解释为真机验证。

接口依据：https://docs.ros.org/en/humble/p/moveit_msgs/
