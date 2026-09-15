# 单臂基础版：FR3 + HKV 示教与抓取 UI

本入口只使用单臂模型、MoveIt、FR3/HKV 驱动和 PyQt 界面。**不构建、不启动 fr3-sim 的 Gazebo 场景、双臂检测任务或相机流程。**

目标环境：Ubuntu 22.04、ROS 2 Humble、x86_64；随附机械臂适配器针对控制柜 3.9.7。
Windows 只能预览界面，不能运行随附 Linux SDK。代码已做离线回归，ROS 构建、实际碰撞阻挡和真机运动仍需目标机验收。

## 1. 安装与启动

在仓库根目录、干净终端中（不要 source 其他双臂/厂商工作空间）：

```bash
source /opt/ros/humble/setup.bash
# 系统尚未安装这些工具时：
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pytest
# rosdep 尚未初始化时执行一次 sudo rosdep init
rosdep update
bash scripts/build_single_arm.sh
source install_single/setup.bash
```

先测试虚拟设备，不连接机械臂：

```bash
ros2 launch fr3_control_panel single_arm.launch.py mode:=mock enable_execution:=true
```

这条命令先打开 UI。点击“连接机械臂与夹爪”才启动单臂模型、MoveIt、RViz 和控制器。MOCK 使用虚拟关节反馈，没有 Gazebo 物理仿真，也不能验证真实夹持。

## 2. 真机配置和启动

首次复制配置（不要覆盖已经标定好的文件）：

```bash
mkdir -p config/local/single
cp -n src/fr3_real_bringup/config/real.example.yaml config/local/single/real.yaml
cp -n src/fr3_real_bringup/config/cell.yaml config/local/single/cell.yaml
cp -n src/fr3_control_panel/config/panel.yaml config/local/single/panel.yaml
```

编辑以下三个文件：

- `real.yaml`：核对控制柜版本后填写 `firmware_version: "3.9.7"`、`driver_package: "fairino_hardware_v3_9_7"`；`controller_ip` 和 `driver_configured_ip` 均填实际 IP，默认 `192.168.58.2`；周期固定 125 Hz。按设备核对夹爪开闭寄存器值、波特率、从站地址、速度和力，再逐项完成 `checks`。不能只把确认项全部改为 true 来跳过验收。
- `cell.yaml`：填写实际桌面、安装位姿、工具/TCP 偏移和碰撞包围盒。默认几何是示例，不是已标定实物。
- `panel.yaml`：填写抓取物的 `payload_size`、`payload_offset`，核对 TCP/参考系。关节闭合坐标默认 0.1，百分比不是实测毫米间距。

只看状态、采点、规划（不激活轨迹控制器）：

```bash
ros2 launch fr3_control_panel single_arm.launch.py mode:=real \
  real_config:="$PWD/config/local/single/real.yaml" \
  cell:="$PWD/config/local/single/cell.yaml" \
  config:="$PWD/config/local/single/panel.yaml"
```

**这不是硬件只读连接**：当前 FR3 插件激活后可能发送 ServoJ 保持指令；UI 未授权运动/`enable_execution:=false` 只限制目标执行。不要在这个驱动运行期间同时用示教器、拖动模式或另一控制程序运动机器人。

现场完成验证后，需要控制机械臂和夹爪时，先断开并退出原 UI，再启动：

```bash
ros2 launch fr3_control_panel single_arm.launch.py mode:=real enable_execution:=true \
  real_config:="$PWD/config/local/single/real.yaml" \
  cell:="$PWD/config/local/single/cell.yaml" \
  config:="$PWD/config/local/single/panel.yaml"
```

UI 操作：

1. 填写 IP（默认 `192.168.58.2`）和夹爪串口（默认 `/dev/ttyACM0`）。IP 必须与核对后的 real.yaml 一致，避免误连另一台设备。
2. 确认现场可启动驱动，勾选启动确认，点击连接。日志区显示后端日志文件路径；启动失败先查看这个文件。
3. 等待六轴角度、开度百分比、TCP 显示有效值。TCP 是**当前关节反馈经 MoveIt FK 和配置的工具模型计算**，不是直接读取控制柜的工具坐标系；XYZ 单位 m，固定轴 XYZ 欧拉角单位 deg。
4. 默认不允许运动。准备操作时勾选“允许运动 / 夹爪操作”；只在启动参数允许执行时该选项可用。

不要额外启动 `ros2_cmd_server`、夹爪独立节点、双臂任务或另一套 MoveIt。这里由一个 controller_manager 加载硬件，每个设备一个驱动实例；界面只发 MoveIt/GripperCommand action。IP/串口锁只能协调本机采用同一锁机制的工程，不阻止别的电脑或第三方程序。RViz 用于看模型/轨迹，不要同时用其 Execute 按钮发目标。

## 3. 目标规划、夹爪和手动采点

- “当前 TCP 填入”：将实时 TCP 复制到目标框。可编辑 X/Y/Z/Rx/Ry/Rz。
- “仅规划 / RViz 预览”：不执行，显示 MoveIt 轨迹。
- “规划并到达”：根据当时最新场景重新规划并执行；规划失败不继续抓取。它不复用先前的预览轨迹。
- “设置夹爪开度”：0% 为闭合、100% 为张开。已有挂载物体时使用“释放物体 / 张开”。
- “采集抓取位姿”“采集展示位姿”“采集途经点”：保存当前有效 TCP，缺失/过期反馈不能采集。双击点位回填，支持修改、删除、上移/下移及 JSON 保存/载入。文件检查参考系和 TCP 名称。

采点按钮只是记录位姿，不切换拖动示教模式。可通过本 UI 的规划运动到达位置后静止采集；若用示教器摆位，应先关闭本工程硬件后端，完成摆位并退出示教器运动控制后再重新连接采集。

抓取顺序：

1. 先采集抓取点、展示点；需要搬运途经点时按所需顺序添加。
2. 使用目标框位姿和尺寸添加桌外工装等盒形障碍物，同名更新。桌子和工具碰撞模型来自 cell.yaml。
3. 在 RViz 检查目标和路径后，点击“① 到抓取点并闭合夹爪”。流程张开 → 规划至抓取点 → 闭合，然后**暂停**。
4. 现场确认实际夹持稳定，以及 panel.yaml 中的负载包围盒和实际物体匹配，再点击“② 确认夹持和负载模型 → 途经点 → 展示”。夹爪停滞也可能是卡阻，不会仅凭停滞自动开始搬运。
5. 物体包围盒成功挂载到 MoveIt 后，按步骤①开始时快照中的途经点顺序逐段规划，最后到展示点并保持夹持。规划、夹爪、场景更新或反馈失败均停止推进。
6. 移动到合适释放位置后点击释放。未确认夹持或闭合结果不确定时，不允许普通机械臂运动；先检查并释放。

本版是人工示教和人工确认的基础抓取，不包含自动物体识别、持续力传感夹持保证、自动交接或严格直线下探。
MoveIt 检查模型里的自碰撞、桌子、工具、已添加障碍物以及挂载负载；不会自动发现现场未建模物体。
目标是笛卡尔位姿，OMPL 路径可以绕行，并非 SDK 直线 MoveL，也不是连续碰撞/动态避障安全系统。不要删除碰撞模型来绕过规划失败。

## 4. 取消、断开与常见问题

- “取消当前操作”或取消勾选允许运动：请求 action 取消，并停止后续流程。**软件取消不是硬件急停**。
- “断开连接”：等待当前后台操作结束，再关闭本 UI 启动的后端。完成前不会重新启用连接；无法确认进程退出时禁止重复连接。
- 关闭窗口时若还有后端，会先断开；提示断开完成后再关闭窗口。
- 驱动不自动使能机械臂、不复位故障、不自动激活夹爪；依厂商现场流程准备设备。串口权限需要系统正确配置。
- 状态可看但运动按钮禁用：检查启动是否带 `enable_execution:=true`，以及 UI 是否勾选允许运动。
- TCP 过期：检查 `/compute_fk`、关节反馈和工具配置；禁止用零值当成当前位姿。
- 点位规划失败：检查可达性、参考系、工具偏移、当前碰撞和目标附近障碍物。
- 更改 IP/串口先断开；更改模型或执行权限需退出并重新启动。更换 IP 后旧点位并不自动重新标定。

## 5. Windows 离线 UI 预览

```powershell
python -m pip install PyQt5 PyYAML
cd src/fr3_control_panel
python -m fr3_control_panel.app --demo
```

无机器人连接、不显示伪造实时状态、运动与采集按钮禁用。

## 6. 目标机验收清单

离线测试：`python -m pytest src/fr3_control_panel/test src/fr3_real_bringup/test -q`。
没有 ROS 时消息序列化测试会跳过，传输替身测试不能替代 ROS 联调。

Ubuntu 上必须实际确认：mock 采点与执行；障碍盒挡住目标时规划失败且无夹爪后续动作；
机械臂/夹爪反馈中断时拒绝操作；执行取消；夹取后人工确认前不搬运；挂载负载参与搬运碰撞；
真机模型与 TCP 校准后低速逐项测试。当前交付不声称已完成这些真机验收。
