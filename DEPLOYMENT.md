# Ubuntu 22.04 / ROS 2 Humble 部署

适配随仓库提供的 Fairino 3.9.7 驱动及 x86_64 SDK。其它固件或ARM主机需先换匹配驱动。
离线测试不等于控制柜、串口、相机与工作站已验收。

## 构建与仿真

在已安装 ROS 2 Humble 的 Ubuntu 上，进入项目根目录执行：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pytest build-essential
# rosdep 尚未初始化的新机器：sudo rosdep init
rosdep update
bash scripts/build_humble.sh
source install/setup.bash
ros2 launch fr3_bolt_inspection_cell bringup.launch.py mode:=gazebo enable_execution:=true
```

构建脚本安装依赖、编译 src 并运行选定测试。`mode:=mock` 仅供模型和规划预览。
厂商包只运行新增的协议/反馈回归测试，不包含其历史源码风格检查。
仓库附 Dockerfile 和 Humble GitHub Actions；当前 Windows 环境无 Docker/ROS，尚未运行容器或 ROS 构建。
无ROS算法测试可在独立venv安装 requirements-test.txt 后运行 `python -m pytest`；运行ROS使用系统Python。

## 现场配置

```bash
bash scripts/init_config.sh /absolute/fr3-config
```

| 文件 | 填写内容 |
|---|---|
| hardware.yaml | 两控制柜IP、实际固件、两个稳定串口设备路径；验证后才设 commissioned=true |
| arms.yaml | 基座安装、法兰与TCP变换；initial是最终回撤目标，须现场验证净空 |
| scene.yaml | 工作台、支架、相机和螺栓尺寸；其它障碍物需增加碰撞几何 |
| inspection.yaml | 桌高、相机外参、展示/交接中心、开合目标和速度；精度应满足现有1mm到位检查 |
| real_feedback.yaml | 指关节开闭端点、双指寄存器零点与接触阈值、持物坐标区间、视觉误差界限 |
| tracker.yaml | 可选ArUco工具的尺寸、标记到零件变换、实测定位误差界限 |

`finger_open/closed` 是与URDF网格一致的单指位移，不是闭合百分比。驱动把寄存器区间线性映射到该坐标；
`open_width/2` 必须等于指关节开端点。指垫有固定CAD偏置，不能假定寄存器100就是100mm净间距。
`held_gap_min/max` 当前按两倍指关节坐标判断，须按实际持物时记录的坐标标定。
X/Y/Z寄存器未校准为力，接触阈值使用去零后的原始范数。位置控制器的max_effort不是硬件力控制；
本版驱动会写配置的目标力到0x0012，不支持或写入失败将报错。

```bash
python3 scripts/check_deployment.py /absolute/fr3-config
```

模板故意不通过校验，示例零阈值不能作为现场参数。

### 离线基座/TCP拟合工具

参考FairinoDualArm的标定工作流，新增两个只处理记录数据、不连接机器人的拟合入口：

```bash
python3 scripts/calibrate_geometry.py registration survey.yaml --max-error 0.001 --output base-fit.yaml
python3 scripts/calibrate_geometry.py pivot pivot.yaml --max-error 0.001 --output tcp-fit.yaml
```

两类输入均要求 `units: metres`。`survey.yaml`提供`source_points`、`target_points`两个等长N×3数组，
至少4个实测对应点，不能共线；若source是机械臂基座、target是world，输出xyz/rpy即对应`arms.yaml`的安装变换。
`pivot.yaml`提供`world_from_tool`的N×4×4矩阵数组：保持工具尖端接触同一固定点，人工安全调整多种姿态，
记录真实反馈计算的位姿，而不是目标位姿。输出`tip_in_tool`是输入tool坐标系中的尖端平移，不包含TCP方向。
当前模型`gripper.tcp_xyz`的父坐标系是`gripper_palm`，只有输入tool也是该坐标系时才能直接填入；
若采集的是控制柜法兰坐标，必须先应用已测法兰到palm的变换。左右夹具不同不能共用一个TCP值，应先扩展分臂工具模型。

工具拒绝退化姿态与超限残差，输出到新文件而不覆盖现场配置，也不自动设置commissioned/calibrated。
拟合残差不是独立精度认证，须用未参与拟合的检查点验证。相机内参与手眼外参仍需按现场相机完成独立标定。

## 真实视觉

已有视觉系统发布 `/inspection/real/object_pose`，类型 `geometry_msgs/msg/PoseWithCovarianceStamped`：

- frame_id=`world`，单位米，时间戳为采集时刻；原点与螺栓模型中心一致，X沿长轴朝头部。
- 完整方向包含绕轴旋转。协方差须有限、对称、半正定、对角线大于零且小于配置质量阈值。
- 位姿来自独立测量，不得由机器人FK乘固定抓取变换或目标指令生成来冒充跟随证据。
- 抓取、抬升、旋转、交接全过程要保持可观测；遮挡超时会取消任务并保留夹持。

桌面点云PCA轴线不足以观察螺栓绕轴滑转。附带 ArUco 节点可用于刚性带标记测试件：

```bash
ros2 run fr3_bolt_inspection_cell aruco_object_tracker --ros-args --params-file /absolute/fr3-config/tracker.yaml
```

使用DICT_4X4_50，尺寸是黑色方块外边长。标记须固定在被测物上，不能贴在夹爪上。
工具不是工业螺栓缺陷识别器。支架/载体若超出螺栓模型，必须加入碰撞几何并重新验证夹持。
相机图像与CameraInfo匹配，输入未去畸变原图；TF须提供采集时刻光学坐标系到world的变换。
配置的不确定度是经独立测试得到的误差界限，不是程序自动估计的定位精度。

三路检测相机仍需图像和CameraInfo，两腕还需深度图。默认逻辑话题为：
`/waist_camera/image_raw`、`/left_d435i/image_raw`、`/right_d435i/image_raw`、各自的 `/camera_info`，
两腕的 `/depth/image_raw`。在real_feedback.yaml的topic_remappings可映射到实际驱动话题。
相机驱动按现场型号另行启动。TF必须解析消息实际frame_id，避免与URDF重复发布冲突的父子变换。
多相机须时间同步；程序等待停稳后的新帧，不会伪造缺失图像。

## 真实执行

示教器中完成安装姿态、TCP/负载、低速与物理停止验证；代码不会自动清故障或使能控制柜。
启动任务要求两爪处于实测空载打开位置。

```bash
bash scripts/start_real.sh /absolute/fr3-config
```

默认关闭轨迹执行，但驱动激活后会进入ServoJ并发送当前位置保持，**不是只读连接**。
反馈验收后退出此次启动，再按现场流程启用：

```bash
bash scripts/start_real.sh /absolute/fr3-config --execute
ros2 control list_controllers -c /left_controller_manager
ros2 control list_controllers -c /right_controller_manager
ros2 action list -t
ros2 topic echo /left/gripper_registers
ros2 topic echo /right/gripper_registers
ros2 service call /inspection/start std_srvs/srv/Trigger '{}'
ros2 service call /inspection/stop std_srvs/srv/Trigger '{}'
```

入口启动双臂MoveIt、两个独立controller_manager及HKV组件、任务节点和检测面板。
每台机械臂只经过该硬件插件；不启动FairinoDualArm的robo_ctrl或独立HKV服务。自动任务进行时不要从RViz/其它面板发送Execute。
同机IP/串口锁阻止本项目第二套双臂启动，不覆盖外部SDK或其它主机。ROS本身没有命令访问控制，多操作者部署应限制ROS网络与命令权限。

机械臂插件检查SDK实时包帧头、帧计数、故障码、急停和碰撞标记；同一帧持续超过100ms即锁存错误并请求StopMotion/ServoMoveEnd。
这能拒绝SDK重复缓存的旧包，不是经过认证的安全控制器；网络断开时停止请求也可能无法送达，必须依靠控制柜和物理安全回路。

交接：接收爪闭合→持续多帧双指接触→独立位姿验证→递交爪打开→接收臂小幅移动验证。
闭合后超时不视为空爪，失败保留夹持；真实模式禁用自动空抓重试和随机物体移动。
`/inspection/stop`是任务取消，不是物理急停。

仍需在Ubuntu完成构建、单臂小步、夹爪映射/保持、双臂规划、测试件视觉、交接逐级验收。
GitHub工作流未运行前不能视为绿色构建。当前记录见 VALIDATION.md。
