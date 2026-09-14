# FR3 双臂检测 sim2real 集成工作空间

集成阶段：保留完整 Gazebo 检测任务；提供双臂 mock/真实硬件调试和左右臂控制面板。真实自动检测尚未接入，不能把仿真持物反馈当成真机反馈。

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
- fairino_hardware_v3_9_7：用现有 prepare_driver.py 从原始驱动生成；修订差异及来源哈希随包保存。
- ros2_hkv_gripper：原 HKV 驱动，闭合行程与真实净开口必须实测映射。
- vendor/FairinoDualArm：参考代码及标定工具，COLCON_IGNORE 排除默认构建。不要与本工程同时启动其 robo_ctrl。

每台机械臂运动只经过本工程的 ros2_control 硬件插件；操作面板与自动任务不得同时下发目标。当前 operator 启动不启动检测任务。

## 未完成与验证范围

当前环境为 Windows，无 ROS 2/WSL 运行环境。离线检查不能代替 colcon、Gazebo 或真机测试。
真实自动检测仍需实现：独立实物位姿输入、带时效和质量指标的夹持反馈、HKV 开口映射、交接证据验证，以及取消/故障后的运动仲裁。现有检测入口继续限制 gazebo/mock。

第三方代码保留各包许可及 THIRD_PARTY 文档；vendor 参考工程的再分发许可尚需确认，因此当前先作为本地集成仓库，不声称已经发布 GitHub。
