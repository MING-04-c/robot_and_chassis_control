# 深度相机点位转换与限力触碰仿真

输入为V54_XYZ_像素测试中的test_pixel_xyz.json，schema=v54_depth_pixel_xyz_diagnostic_v1。该文件是手选像素数值测试，不表示已验证的漆膜缺陷。相机光学坐标XYZ为[-5.370267, -3.820747, 275]毫米，不是base_link坐标。

入口始终使用GenericSystem假硬件、127.0.0.1及独立ROS域191，没有simulation=false或真机IP入口。每次发轨迹及启动检测前检查硬件插件。默认相机光学坐标临时与模型terminal_link_1轴对齐；这不是实测手眼外参，不能据此在真机触碰。

## 运行

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control pixel_point_simulation.launch.py
```

启动后自动执行，无需另调服务：

1. 等待假硬件、关节反馈、MoveIt与检测接口。
2. 规划到仿真参考关节姿态，捕获此时模型相机和探头TF。
3. 把相机点从毫米转米，再由base_T_camera变换到base_link；固定这个基座点，不随机械臂随后运动漂移。
4. 保持参考姿态的探头朝向，利用tool0_T_probe补偿探头长度，求出tool0目标。
5. 复用detect_control先规划至目标上方20毫米，启用导纳，沿基座-Z以2毫米/秒接近。
6. 模拟探头到达目标面后发布21N接触事件，检测器的20N阈值触发停止推进，保持5秒，直线撤离并返回记录的六轴原位。

RViz红点是检测点，橙点为预接近点，蓝线为实际仿真探头轨迹。终端出现COMPLETE表示检测器报告完成并观测到HOLD；失败显示FAILED和原因。不跳过碰撞检查、限位、力数据新鲜度或规划失败。

需要循环观察时传 `repeat:=true`，需要无界面时传 `launch_rviz:=false`。Ctrl+C退出本次仿真。自定义输入可传 `points_file:=/完整路径/test_pixel_xyz.json`。

配置：src/robot_task_control/config/tests/pixel_point_simulation.yaml。接触阈值contact_force=20N，模拟力simulated_contact_force=21N，保护阈值40N，保持5秒。若修改阈值，runner和detect_controller两段配置需一致。参考关节是仿真示例，不改真机初始位置。模型安装轴到光学轴的临时旋转在mount_to_optical_rpy_deg配置，单位度。

## 验证范围

已对输入点检查毫米/米换算与像素投影一致性，使用当前ls65及新增末端模型进行离线FK/有限关节范围内数值IK检查。当前临时外参假设下，基座探头目标约[-0.671502, -0.092043, 0.273002]米，目标与20毫米预接近点均找到数值逆解。

离线数值逆解不等于无碰撞路径。运行时仍由MoveIt检查全模型碰撞并执行规划。

力来自明确的仿真接触事件，不是真实传感器，也不是Gazebo物理材料/接触动力学。它用于检查导纳接近、20N停止、5秒保持和回原位状态流程，不能验证真实20N接触精度。实机还需要光学外参、拍照时的机器人位姿及真实力传感器数据。

运行报告：/tmp/pixel_point_simulation_report.json，保存转换后的点位、临时外参假设、接触阶段/滤波力和执行结果。
