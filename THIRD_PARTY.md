# 集成来源与分发边界

- `fr3_bolt_inspection_cell`、`fr3_dual_bolt_cell`：基于用户提供的 fr3-sim 工作区，保留任务、几何和双臂 MoveIt 规划。
- `fr3_control_panel`、`fr3_real_bringup`：基于用户提供的 fr3-hkv-control-panel；本版自动检测入口不同时启动人工控制面板。
- `ros2_hkv_gripper`：来自用户提供的桌面源码，本版修改串口事务校验、物理开口映射、反馈发布和关闭保持。原包 README、package.xml、代码头部的许可声明不一致，尚待原作者确认。
- `fairino_hardware_v3_9_7`、`fairino_msgs`：来自用户提供的 frcobot_ros2-v3.0.0_robotV3.9.7。保留 SDK 头文件及 Linux x86_64 的 libfairino.so.2.3.7，本版继续修订独立IP连接、伺服生命周期与实时反馈检查。
- `FairinoDualArm-main`：参考其双臂连接、真实状态和标定工具思路；没有把其直接运动节点放入启动图，也未打包整个参考目录。新增 calibration.py 独立实现点对刚体配准与多姿态定点TCP拟合，未照搬原脚本的路径、数据或手眼求解流程。

各包原有来源说明保留；“本包不带 SDK”只适用于对应子包，本工作区确实包含用户提供的版本驱动和 SDK。
第三方代码、网格与厂商二进制不因本项目集成而获得新许可。当前交付用于用户已有资产的本地集成；若上传，先采用私有仓库。
公开分发或商用再分发前应确认许可冲突、厂商SDK再分发权限及权利人声明。`vendor/`不参与构建、不进入提交或交付归档。
