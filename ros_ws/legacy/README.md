# 两机部署旧代码归档

本目录有 `COLCON_IGNORE`，不参与 colcon 构建。

- `eli_to_chassis_control`：原笔记本联合任务控制器。
- `masc_user_serial`：原树莓派 TCP 状态、命令和摄像头桥。
- `computer_pi`：原笔记本 TCP 桥和远程画面接收端。
- `ChassisControl.srv`：原通信桥服务定义。

新入口为 `ros2 launch robot_task_control robot_system.launch.py`。
旧包已移出 src，串口包不再生成旧底盘接口，也不再安装 computer_pi 文件。
不要 source 以前笔记本或树莓派的 install；旧安装产物可能仍注册旧包。

原 `masc_user_serial` 的相机图像 TCP 发送和接收工具已复制到 `robot_task_control` 独立使用，用于 HK 照片外传；底盘/机械臂控制桥继续归档。
