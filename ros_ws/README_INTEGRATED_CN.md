# 本机整合入口

新的联合控制包为 [robot_task_control](src/robot_task_control/README.md)，部署步骤、参数、流程说明和无硬件测试见该文档。

- 机械臂：保留 eli_cs_robot_* 的驱动、MoveIt、导纳与外设控制代码。
- 底盘：保留 masc_chassis_* 的 CAN SDK、ROS 驱动和 PID，使用本机类型化任务服务与状态。
- 任务：新增 robot_task_control，以原 eli_to_chassis_control 的 YAML 和机械臂逻辑为基础。
- 两机控制通信：归档到 [legacy](legacy/README.md)，不参与构建；HK 照片发送单独恢复，沿用 MIMG 协议。
- 摄像头：HK 拍照与照片外传，USB 蓝线循迹，独立图像话题和速度选择器。
- 构建：scripts/build_raspberry_pi.sh 从源码编译机械臂 SDK 与必要 ROS 包；scripts/raspberry_pi_env.bash 设置运行环境。

旧 README 中笔记本路径、两机 TCP 启动方式和通信话题属于历史部署说明；整合后的启动以新包文档为准。
