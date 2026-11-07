# 相机检测点与探针触碰测试：TCP / 本地点位

同一个入口 `camera_detection_real.launch.py` 提供两种模式，默认 `target_source:=tcp`。包含真机驱动、MoveIt、真实力传感器、检测器和串口测厚。`camera_probe_touch` 默认关闭导纳，使用普通 MoveIt 直线规划加 20N 力保护；通用 `camera_detection_real` 默认保留导纳。**在TCP模式发送本次匹配结果后会自动执行真机运动；local模式就绪后自动运动。**

## 1. 默认：采图、接收主机点位、自动触碰

退出旧的仿真、机器人控制、独立相机预览和9010通讯节点。将机器人放在拍照位置，探针保持未接触。麒麟使用新版完整通讯工具目录，先运行：

```bash
cd /实际路径/industrial_pc_kylin_tcp
python3 listen_robot.py
```

树莓派：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control camera_detection_real.launch.py
```

默认两种点位来源均使用普通 MoveIt 加力阈值停止，不启动导纳节点。以下为本地点位测试：

```bash
ros2 launch robot_task_control camera_detection_real.launch.py \
  task_profile:=camera_probe_touch touch_control_mode:=moveit_force_guard \
  launch_rviz:=false
```

默认机器人IP为169.254.199.150，Pi为169.254.199.100，麒麟为169.254.199.120。机器人地址可传 `robot_ip:=...` 修改。启动后自动执行以下流程，不需要 start_task、start_capture 或 enable 服务：

1. 确认真机外控、控制器、真实力与新鲜TF。连接连续稳定后默认等待5秒加载控制系统。
2. 当前姿态稳定2秒，冻结基座到相机光心及探针的变换。
3. 按需开启HK，等画面稳定拍一张并关闭；开启深度相机采RGB/原始深度并关闭，再通过原通道发给主机。
4. 等待主机检测结果。采图和等待期间机器人保持原位；位移超过1mm或方向变化超过0.5°，本轮作废。
5. 按本次 `capture_manifest_sha256` 匹配结果。其他照片、DEMO忽略；本次HOLD明确取消。主机可在传图尚未结束时返回结果，程序会暂存，传输成功后再处理。
6. 用冻结的拍照变换，把光学毫米坐标转为基座米制探针目标，保持拍照时探针朝向。清空旧目标、保存拍照关节，检测后返回位使用配置行驶姿态；确认整组新点入队后自动检测。

主机检测程序对本次图片生成正常 `paint_v54_pi_remote_result_v1` 结果JSON后发送：

```bash
python3 send_points.py --file /实际路径/result.json
```

坐标字段是 `points_xyz_camera_mm`，每个点包含唯一point_id与三个毫米值；`coordinate_frame=CAMERA_PINHOLE_PRELIMINARY`，`status=OK_3D_POINTS`，`depth_source=MEASURED`。沿用协议中的观测标记，不把主机消息声明为机器人运动命令。执行授权来自本机启动的本测试任务。

哈希从主机 `received/.capture_index.json` 中本轮记录的 `rgbd.capture_manifest_sha256` 读取。两端图片结构为 `2026.10.30.20.29/hk/`、`2026.10.30.20.29/RGB/`，同轮时间前缀相同。完整协议和格式在麒麟工具目录的TCP通讯说明中。

如果使用已有V54手选像素测试程序，必须针对这次新图重新生成测试点，再发送：

```bash
python3 send_points.py --pixel-test --file /实际路径/test_pixel_xyz.json \
  --rgb-file received/2026.10.30.20.29/RGB/20.29.12_345_rgb.png
```

源RGB/深度哈希及深度比例必须与本次图像相符。旧样本和演示点不能用于新照片。手选实测点可用于触碰测试，不表示已验证漆膜缺陷。

## 2. 直接给本地点位

```bash
ros2 launch robot_task_control camera_detection_real.launch.py target_source:=local
```

此模式不启动TCP或拍照相机。点位在 `src/robot_task_control/config/camera_detection_real.yaml`：

```yaml
camera_detection_task:
  ros__parameters:
    camera_points_mm: [-5.370267016871055, -3.820746759094509, 275.0]
```

多个点连续填写 `[x1,y1,z1,x2,y2,z2,...]`，单位毫米。它们是相对于本次启动时相机光学坐标系的测试几何，不能认为旧照片中的物体就在这些新目标处。接触参数与本地点位仍集中在同一个YAML。

## 3. 两种模式共用的控制

Joint_1 / terminal_link_1 已由用户确认为实际光心及光学轴：图像右X、图像下Y、镜头前Z。探针触点是terminal_link_2；detect_control统一把探针目标补偿回tool0。

每点由MoveIt进行碰撞与可达性检查，预接近点沿tool0局部+Z的反方向距目标20mm。预接近到位后冻结此轴，沿+Z以2mm/s执行低速直线；20N触发后取消轨迹，保持5秒并采集厚度，撤离并确认实际返回记录的六轴原位，再回传本点结果、测下一点。默认合力保护40N。名义目标之后最多再下探50mm，未接触即停止并进入受保护的撤离流程；普通 MoveIt 模式全程不启用导纳。

20N是停止推进的限力阈值，不是恒力伺服。外参使用已确认的模型变换，不额外旋转相机轴。TCP模式采用整段静止校验来绑定拍照位姿，不是运动期间的图像时间插值。

## 4. 配置与复用

- `target_source:=tcp`（默认）或 `local`：只通过此启动参数切换模式。
- `touch_control_mode:=moveit_force_guard` 为默认限力接近；显式设为 `admittance` 可保留旧导纳流程。
- `task_config:=/绝对路径/配置.yaml`：检测点、20N阈值、速度、保持、等待上限。
- `capture_config_file:=/绝对路径/配置.yaml`：采图与传输参数，默认 `config/capture/final_capture.yaml`，仅TCP使用。
- `launch_rviz:=false`：减少显示开销。
- `start_arm_driver:=false`：复用已有驱动；完整控制栈也已有时再加 `start_arm_control:=false`。检测器与TCP不要重复启动。
- `start_thickness_detector:=false`：已有测厚串口节点时复用；默认true，115200/8N1，接触后触发采集。

默认采集/传输上限240秒，等待本次主机结果300秒；失败或超时不回退使用本地YAML点。每轮正常完成一次；行程用完、目标不可达或没有有效厚度回包时，在安全回原位之后TCP模式最多额外重拍取点3次（max_reacquire_retries）。主机须保持point_id稳定、哈希对应新照片，已成功点不会重测。原位只在第一批记录；本地固定点失败后不会假装重拍。过载、碰撞/安全锁定、断流或回位失败保持停止，不进入此重试。

## 5. 状态、取消和验证范围

```bash
ros2 topic echo /task_controller/status
ros2 topic echo /detect_controller/status
```

TCP状态：WAIT_READY → CAPTURE → WAIT_RESULT → CLEAR_TARGETS → RECORD_HOME → WAIT_TARGETS → STARTING → RUNNING → DONE。local略过CAPTURE和WAIT_RESULT。状态含模式、本轮run_id、清单哈希与已完成点数，麒麟监听窗口持续显示。采图不会占用常驻预览或点云；相机关闭后再触碰。

麒麟可 `python3 start_task.py --action stop` 取消本任务；树莓派可调用 `/task_controller/stop`。直接取消检测器仍使用 `/detect_controller/cancel`。不支持在本任务里调用start_task或start_capture来重启本轮；默认自动执行，不增加手动开始步骤。

记录保存在 `~/.ros/camera_detection/<run_id>.json`，包含冻结位姿、结果ID、点位与阶段。终端会打印换算后的基座探针目标。

退出时在启动launch的终端按Ctrl+C。先停止任务调度并取消检测，只清理本任务创建的相机进程组，不终止另行运行的驱动。保留有限等待及launch的3秒SIGTERM、5秒SIGKILL升级。进程退出不代表真机已确认停止，仍运动时以示教器停止为准。

开发验证使用替身节点、冻结位姿及回环通讯检查两模式、结果关联、坐标换算与取消；没有替用户实际执行真机触碰。

逐点回位结果：`/task_controller/detection_result`，树莓派保存在`~/.ros/camera_detection/point_results.jsonl`；TCP桥接持久化后通过9010状态查询提供最近100条。更新后的麒麟`listen_robot.py`自动补收、按ID去重并保存在output_dir/detection_results/，显示最新读数和成功/失败。

## 双关节姿态中转

两份相机profile启用`joint_postures_enabled: true`。共用度制配置在`eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`：行驶位[-45,15,-71,4.4,148,75]，中转位[-45,0,0,0,90,0]。中转位不是最终拍照/触碰姿态，不将它的近水平tool0 +Z当作向下。首次拍照仍在用户准备好的原拍照姿态；开始目标规划时先经过中转，再到预接近。检测后反向撤离、关节中转、关节回行驶位；实际回行驶位才发布逐点结果。若需重拍，先从行驶位经过中转，恢复本轮保存的原拍照关节，再冻结新照片变换。共用MoveIt场景中base_link的Z≤0.08m为地面禁入区，只有固定安装底座例外。

双向仿真检查：`ros2 launch robot_task_control joint_posture_test.launch.py`；默认假硬件独立域190，可显式`simulation:=false`连接真机，自动第一→第二→第一，不启动真实相机、力传感器或底盘。

真机双关节往返：先退出重复的机器人launch，再执行`ros2 launch robot_task_control joint_posture_test.launch.py simulation:=false robot_ip:=169.254.199.150`。默认包含真机驱动（headless、自动上电/释放抱闸）、MoveIt、8cm地面和RViz；不启动相机/底盘/力传感器。默认速度缩放10%。等待外控、真实硬件插件、scaled轨迹控制器、安全状态和地面就绪后再等5秒；当前位置不是行驶位时先规划到行驶位，再中转、返回行驶位。启动后自动执行，无需调开始服务。已有驱动可加`start_arm_driver:=false`，已有MoveIt再加`start_moveit:=false`，但共用地面节点及安全监控必须存在。真机报告`/tmp/joint_posture_real_report.json`。本次开发验证不执行实体机械臂。

## 普通 MoveIt 模式的边界

到预接近点、撤离和回原位仍由 MoveIt 规划；接触下探是一次计算的完整无碰撞直线，执行期间监测力并取消。
这减少持续导纳/增量IK计算，但 MoveIt、碰撞场景和底盘驱动仍常驻，实际CPU收益需测量。
避障仅针对规划场景中已有的模型和障碍；深度图片不会自动成为动态障碍。
20N是滤波轴向力的停止触发值，40N合力保护额外检查未滤波去偏数据；取消与机械停止有延迟，不能承诺峰值不超过20N。
保持5秒属于位置保持和测量窗口，不是恒力维持。没有达到接触阈值时仍受名义点外50mm行程限制。
本模式当前只有静止零偏采样，没有完整工具重力补偿；大幅改变工具朝向后应核对空载读数，不能把重力变化当作接触力。
