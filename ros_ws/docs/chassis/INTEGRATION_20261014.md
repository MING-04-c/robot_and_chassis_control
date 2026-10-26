# 底盘更新整合记录（2026-10-14）

更新源为项目根目录的 `masc_chassis/chassis_ros_ws`。所有运行代码统一放入 `ros_ws/src`，原导入目录在验证后移除。

## 更新范围

- `masc_chassis_user_control`：新版近端中线/前瞻循迹、平滑控制、参考标定、调试画面、自适应 USB 曝光、键盘控制、拖车会话、配套配置和测试。
- `camer_diver`：新版暗蓝胶带识别阈值（value_min=4）及测试和说明。保留当前 video_device、camera_frame 启动参数。
- `masc_chassis_can_sdk`：拖车专用会话关闭周期运动帧的配置、实现和回归测试。
- `masc_chassis_ros_driver`：低速滤波累积/自动解锁修正、command.transmit_enabled 支持与自动驻车测试。
- `robot_task_control`：新版循迹继续使用私有速度话题；USB 识别、预览与曝光均映射到 /line_camera；新增独立 keyboard_teleop.launch.py 入口。

## 保留的整合行为

保留 TaskControl/TaskStatus 类型化任务接口、执行编号、PID 与循迹速度选择、三种独立任务流程、单次 HK 拍照和照片外传。更新源内的旧 TCP 底盘任务接口没有恢复。原 masc_user_serial 与归档版本相同；HK 驱动底层也没有新差异，不覆盖当前拍照服务改动。

相同的 CAN 脚本和 systemd 配置继续复用，项目级旧说明合入本目录供参考；旧部署示例中的 TCP 控制桥仅代表原部署方式，当前启动以 robot_task_control/README.md 为准。

导入前源代码备份在 `/tmp/masc_chassis_before_merge_20261014.tar.gz`，不包含构建输出与源仓库 .git；临时目录不是长期备份。

## 验证结果

5 个相关包重新构建成功；CAN SDK 协议/运行测试、相机与蓝线识别、自动驻车回归、循迹控制/ROS、启动作用域、曝光控制/ROS、拖车会话和键盘伪终端测试全部通过。原有 7 项任务流程测试通过，包含三种独立流程；新增统一启动检查确认 USB 图像、预览、曝光与私有速度话题一致。迁移清单核对后已移除项目根目录的 masc_chassis。尚未连接真实相机和底盘进行实车验证。
