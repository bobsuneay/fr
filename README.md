# FR3 双臂检测 sim2real 集成工作空间

本版保留完整 Gazebo 检测任务，已接入双臂真实自动检测后端：MoveIt规划、HKV GripperCommand、持续双指寄存器接触验证、独立视觉位姿与交接验证。
适配Ubuntu22.04/ROS2 Humble/x86_64/Fairino3.9.7。完整步骤见 [DEPLOYMENT.md](DEPLOYMENT.md)。

## 真实自动检测部署入口

```bash
bash scripts/build_humble.sh
bash scripts/init_config.sh /absolute/fr3-config
# 填写实际IP、串口、标定和接触阈值后：
python3 scripts/check_deployment.py /absolute/fr3-config
bash scripts/start_real.sh /absolute/fr3-config
# 反馈验收、退出上次启动后，按现场流程启用轨迹：
bash scripts/start_real.sh /absolute/fr3-config --execute
```

相机驱动需按现场型号启动。提供标准PoseWithCovarianceStamped接口及可选ArUco测试件跟踪工具，见部署文档；不会用机器人指令生成虚假物体跟随证据。
以下控制面板入口为独立人工调试，不能与自动检测同时启动。

## 构建（Ubuntu 22.04 / ROS 2 Humble）

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

完整仿真检测：

```bash
ros2 launch fr3_bolt_inspection_cell bringup.launch.py mode:=gazebo enable_execution:=true
```

双臂 mock 与面板（选择一侧操作）：

```bash
ros2 launch fr3_sim2real operator.launch.py mode:=mock side:=left enable_execution:=true
```

双臂真机调试：先按 `src/fr3_dual_bolt_cell/docs/COMMISSIONING.md` 完成版本、基座、TCP 和夹爪校准，复制其 hardware.example.yaml 为本地配置。之后启动：

```bash
ros2 launch fr3_sim2real operator.launch.py mode:=real side:=left hardware:=/absolute/path/hardware.local.yaml enable_execution:=false
```

execution=false 只禁用轨迹控制器，硬件插件仍可能发送保持指令，不能作为只读连接使用。首次执行需现场验证后显式启用。

## 集成边界

- fr3_bolt_inspection_cell：原任务与双臂 MoveIt 规划保留。
- fr3_dual_bolt_cell：两台独立 controller_manager，带前缀关节，共享规划场景。
- fr3_control_panel/fr3_real_bringup：来自 fr3-hkv-control-panel，面板通过统一 MoveIt/夹爪 action 操作。
- fairino_hardware_v3_9_7：在prepare_driver生成版本上继续修订伺服生命周期和真实反馈帧检查；旧diff只描述初始适配，不是本版完整差异。
- ros2_hkv_gripper：原 HKV 驱动，闭合行程与真实净开口必须实测映射。
- FairinoDualArm：借鉴连接隔离、状态读取及标定工作流；本版新增独立的离线基座/TCP拟合工具，不打包vendor参考目录。不要同时启动其robo_ctrl。

每台机械臂运动只经过本工程的 ros2_control 硬件插件；操作面板与自动任务不得同时下发目标。当前 operator 启动不启动检测任务。

## 验证范围

当前环境为Windows。离线算法、模型及真实后端状态机测试已通过；HKV协议、FR3反馈看门狗C++测试通过。
ROS2编译、Gazebo运行和真机运动尚未在本机验收，不能据此声称即插即用。提供Humble CI和Dockerfile用于目标平台构建，详见VALIDATION.md。
真实入口已支持mode:=real，但要求有效标定与独立视觉输入。闭合超时、反馈过期、夹持不确定会中止自动推进；真实自动重试被禁用。

第三方代码保留各包许可及 THIRD_PARTY 文档；vendor 参考工程的再分发许可尚需确认，因此当前先作为本地集成仓库，不声称已经发布 GitHub。
