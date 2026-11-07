# 任务控制包

本包负责组织任务顺序、速度选择及按需采集。机械臂、底盘、相机、力传感器驱动仍由各自功能包提供；工控机 TCP 通讯唯一实现在 `eli_cs_robot_serial/device/industrial_pc/`。

所有操作命令集中在 [机械臂与底盘 ROS 控制命令](../../机械臂与底盘ROS控制命令.md)，本文件只解释目录与入口，避免两份说明不断追加后互相矛盾。

## 常用入口

| launch 文件 | 用途 | 开始方式 |
|---|---|---|
| arm_admittance_only | 仅机械臂导纳 | 就绪后自动 |
| line_follow_only | 前进记录、reverse返程；round_trip:=true 按任务顺序自动往返 | 就绪后自动；auto_start:=false 仅等待 |
| keyboard_teleop | 底盘键盘控制 | 键盘 |
| line_follow_arm_test | 整体联动：行驶姿态、逐站循迹、四点动作、中转等待 | 默认自动；remote_control:=true 等主机开始 |
| camera_capture_remote | 只采图发送，不加载运动驱动 | 麒麟 start_capture.py |
| camera_detection_real | 相机点探针检测，默认TCP、可选本地YAML | TCP收到匹配结果后自动；local就绪后自动 |

表中六个常用入口均以 `.launch.py` 结尾。整体联动统一使用 `line_follow_arm_test.launch.py`：默认就绪后自动执行；`remote_control:=true` 加载TCP并等待麒麟 `start_task.py`。可显式传 `auto_start:=false` 暂停自动开始。

旧 `line_follow_arm_remote.launch.py` 已合并删除；旧两秒导纳＋循迹入口及其运行配置已移除。

## 目录

```text
launch/                 常用任务入口
  internal/             robot_system、detection_system：共用驱动与控制栈
  tests/                像素点仿真入口
config/                 常用任务流程及本地检测点
  capture/              最终拍照、独立采集及旧 HK 采集参数
  tests/                到达测试、像素仿真配置和样本
robot_task_control/
  workflow_controller.py    YAML 工作流执行器
  velocity_mux.py           底盘 PID/循迹速度选择
  camera_detection_task.py  两模式真机限力测试
  final_capture.py          顺序开启相机、稳定采图、关闭后发送
  testing/                  仿真与旧到达测试节点
rviz/                   检测显示配置
test/                   本包回归测试，非任务启动入口
```

`internal/robot_system.launch.py` 供单步和整体联动复用；`internal/detection_system.launch.py` 供探针检测复用。两者不是日常任务入口。旧综合示例及示例底盘任务表已删除，启动核心默认使用当前整体联动配置。`config/tests/` 只供仿真及内部调试使用。

海康转发壳 `robot_task_control/hk_image_transfer.py` 已移除：启动核心直接调用串口通讯包的 `industrial_pc_hk_transfer.py`。旧 `ros2 run robot_task_control hk_image_transfer` 名称仅通过 entry point 直接指向唯一实现，未保留第二份代码。原来引用不存在接收工具的 HK 测试已迁到通讯包目录并修正引用。

## 各流程的职责

- 工作流控制器：驱动就绪检查、任务顺序、失败/取消处理；自定义 `chassis_execute` 流程需自行提供 `task_table`，实际位移参数仍在底盘驱动包。
- 速度选择器：只转发被选中的 PID 或循迹速度，切换/输入过期时发零速。
- 相机点检测：TCP模式稳定采图并绑定本次结果，local模式直接使用YAML；两者冻结相机/探针位姿，将毫米点变成基座目标；detect_control 负责 tool0 补偿、碰撞规划、限力接近、保持和逐点回原位。
- 最终采图：复用 HK/深度驱动及串口包的传输实现，不启动常驻预览和点云。
- 主机通讯：start/stop/resume/capture、状态、检测点观测和最新厚度缓存；不把普通点位回包直接变成运动指令。

`camera_detection_real` 默认接收本次照片的主机点位并自动触碰；`target_source:=local` 保留原本地给点测试。它与整体联动任务二选一，不能同时占用机器人、相机或9010。

## 测试与进一步说明

树莓派联合任务的 CPU、内存、温度、调度和每个 ROS 进程采样流程见
[资源测试说明](../docs/tasks/RESOURCE_PROFILE_CN.md)。

没有重复删除不同职责的测试：`test_camera_point_tasks` 检查坐标/本地流程/退出，`test_arm_description_launch` 检查模型与启动组合，`test_arm_jazzy_startup` 检查假硬件加载，`test_device_startup` 检查设备就绪门控，`test_local_control` 检查工作流和速度选择，`test_final_capture` 检查采集稳定性和进程边界。涉及 ROS 联调的测试按文件单独运行，不连接真实运动硬件。

- [相机点两模式真机测试](../../docs/tasks/CAMERA_DETECTION_REAL.md)
- [循迹各站点与机械臂姿态联动](../../docs/tasks/LINE_FOLLOW_ARM_TEST.md)
- [像素点触碰仿真](../../docs/tasks/PIXEL_POINT_SIMULATION.md)
- [主机通讯实现](../eli_cs_robot_serial/device/industrial_pc/README.md)

首次目录调整后需清除旧安装路径中的同名 launch，避免 ROS2 递归查找出现重复；本次工作区已完成迁移。其他机器更新代码时先备份自行修改过的安装配置，再只清理 `build/robot_task_control`、`install/robot_task_control` 并重新构建该包。本轮另已清理被删除入口及旧示例的安装副本。旧 remote 命令改用 `line_follow_arm_test.launch.py remote_control:=true`；旧 detection_test 是内部检测栈，不再作为常用入口。

记录路线返回复用 `line_follow_only.launch.py`。前进默认记录 `/wheel_odom` 到 `~/.ros/line_follow/last_route.json`，倒车自动选用 `config/line_follow_return.yaml`，到记录起点收到 `reverse_route_complete` 才完成。该回程允许最多900秒，故障/偏离/超时保持停车；前置图像丢线不作为完成条件。保持前进停车时的位置和朝向再启动倒车，详情见循迹包说明。

## 自动循迹往返与返程路线封存

复用 `line_follow_only.launch.py round_trip:=true`，流程配置为 `config/line_follow_round_trip.yaml`。前进途中站等待新继续消息，最终横线结束前进；随后等待底盘连续停稳、自动执行 `save_line_route` 封存路线，再执行中间步骤，最后由 `line_follow` 的 `drive_direction: reverse` 开始返程。倒车步骤设置 `skip_intermediate_markers: true` 自动继续途中站，并设置 `return_to_start: true` 核对已记录出发站；仅在原出发点完成返程。若出发点需要视觉横线确认，应事先设置出发停止横线。

封存副本位于 `~/.ros/line_follow/snapshots/`；`/task_controller/status` 的 `route_checkpoint_file` 和 `/line_follow/status` 的 `route_snapshot_file` 可查看本轮文件。等待机械臂/TCP期间不追加副本，不清零里程计，也不停止底盘驱动。等待期间观测到底盘相对封存位置移动超过2.5cm、转动超过0.05rad或里程计异常时拒绝自动倒车；这些是保护界限，不是定位精度保证。

当前测试在封存后预留2秒 `delay`。实际联动时，在 `save_line_route` 之后、reverse步骤之前插入机械臂或采集/TCP步骤，任务控制器只会在上一步确认完成后推进。测试入口只启动底盘、循迹相机及任务控制器，不会额外启动机械臂或TCP进程。命令及途中站继续方式见工作空间控制命令文档。

## 相机坐标到探针触碰

`config/camera_probe_touch.yaml` 保存当前测试点 `(15,15,160)` 毫米（以YAML实际填写为准）。复用已有入口，不另增一套执行节点：

```bash
ros2 launch robot_task_control camera_detection_real.launch.py task_profile:=camera_probe_touch launch_rviz:=false
```

默认本地点模式，启动即执行；需先将机械臂停在对应拍照姿态。冻结相机 TF，将点转换成基座下探针目标，再补偿到 tool0；各到位目标沿用启动朝向（容差1度），中途由规划器选择姿态；预接近到位和下探仍检查实际朝向。预接近点沿 tool0 局部 +Z 的反方向距目标20mm；到位后冻结该轴，沿 +Z 以2mm/s接近。轴向力绝对值达到20N后停止推进、保持5秒、反向撤离并返回启动时的关节原位。未达到20N时继续越过名义目标，额外行程最多50mm。行程用完或不可达时，先确认停止，安全撤离回原位；TCP模式重拍取点后重试，最多3次。本地模式回位后停止，须更新坐标。过载或数据异常仍停止，不强行回位。每个点都返回同一个原位后才执行下一点。点位、速度、力阈值均在同一配置修改；不会启动相机/工控机通讯/底盘，默认包含串口测厚，接触保持时采集，实际返回原位后发布逐点结果；已有测厚节点可加 `start_thickness_detector:=false`。

末端模型现采用主目录 `robot (2)/robot.urdf`：相对tool0安装旋转-90°，沿tool0 -Z平移2.7mm靠近法兰；新版Joint_2相机映射为 `terminal_link_1`，Joint_3探针映射为 `terminal_link_2`。已撤销额外180°光学补偿。原位、力阈值及局部+Z接近逻辑保留；历史基座目标需从当前拍照姿态重新生成。

逐点结果由 `/task_controller/detection_result` 经公共TCP桥接交给麒麟 `listen_robot.py`，保存在 `detection_results/`。重拍时point_id须保持稳定，照片清单哈希必须更新；已成功的点不重复测。机械臂原位只在首次执行前记录，重拍不会改写。详见工作空间命令文档及 `eli_cs_robot_serial/device/detactor/README.md`。

## 两组关节姿态测试

`ros2 launch robot_task_control joint_posture_test.launch.py`自动以假硬件执行“行驶位→检测中转位→行驶位”，先验证两端碰撞及双向规划，仿真固定独立ROS域190，`simulation:=false`启用真机模式。两组度制姿态集中保存在`eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`，结果见`/tmp/joint_posture_test_report.json`。

相机检测profile启用`joint_postures_enabled`：规划预接近之前经过检测中转位，撤离后经过中转位再按关节角回行驶位。最终检测朝向继续来自实际拍照姿态，不用中转姿态代替。首次记录时同时保存拍照关节，用于失败后的经中转位重拍恢复。MoveIt共用入口自动配置base_link下8cm地面，仅固定安装底座允许相交。

真机双关节往返：先退出重复的机器人launch，再执行`ros2 launch robot_task_control joint_posture_test.launch.py simulation:=false robot_ip:=169.254.199.150`。默认包含真机驱动（headless、自动上电/释放抱闸）、MoveIt、8cm地面和RViz；不启动相机/底盘/力传感器。默认速度缩放10%。等待外控、真实硬件插件、scaled轨迹控制器、安全状态和地面就绪后再等5秒；当前位置不是行驶位时先规划到行驶位，再中转、返回行驶位。启动后自动执行，无需调开始服务。已有驱动可加`start_arm_driver:=false`，已有MoveIt再加`start_moveit:=false`，但共用地面节点及安全监控必须存在。真机报告`/tmp/joint_posture_real_report.json`。本次开发验证不执行实体机械臂。

循迹联动测试已接入行驶/中转关节姿态：每个横线站执行四点动作，回中转等待；途中站收到下一次开始后，先回行驶位确认到位，再继续。最终站动作后保持中转。默认不拍照；配置共用`station_steps`，详见`ros_ws/docs/tasks/LINE_FOLLOW_ARM_TEST.md`。单独循迹入口不启动机械臂。

自动往返充电测试复用`line_follow_arm_test.launch.py test_profile:=return_charge`：行驶位→前进首条有效横线→停稳封存→中转位→等待5秒→行驶位→倒车充电→停稳结束。无需逐步调服务。配置为`config/line_follow_arm_charge_test.yaml`；默认不启动力传感器和海康/深度相机，默认加载状态监视/TCP桥。详见正式联动流程文档。


## 单独循迹：离桩、首条横线等待5秒、返程充电

先退出其他底盘/联合任务，避免同时向速度话题发送指令。启动后自动执行，无需调用开始服务：

```bash
source /opt/ros/jazzy/setup.bash
source ~/project/robot_and_chassis_control/ros_ws/install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py \
  round_trip:=true charging_mode:=true \
  line_show_window:=true line_stop_marker_target_ratio:=0.5
```

复用已有launch，选择 `config/line_follow_charge_test.yaml`：
底盘停稳→前进循迹离桩→第一条有效横线停稳→封存路线→等待5秒→沿记录路线倒车充电→停稳结束。
起步出发线按原规则忽略；到站横线既可以是途中横线，也可以是末端横线。
前进离桩仍为最高0.02m/s、连续3帧不充电后恢复正常循迹；超过30cm或25秒仍充电则锁停。
倒车保留视觉回中、横线定位、10cm额外搜索限制，连续充电反馈确认才完成。
不会启动机械臂、MoveIt、RViz、TCP或其他相机；整个往返期间USB循迹相机和里程计保持工作。
`round_trip:=true charging_mode:=false` 继续使用原来的“最终横线往返”流程。

标准路线的低速录制、命名存储、版本/质量检查及重放流程见工作区 `docs/tasks/STANDARD_ROUTE.md`。默认 `navigation_mode=vision` 保留原来的循迹入口。
