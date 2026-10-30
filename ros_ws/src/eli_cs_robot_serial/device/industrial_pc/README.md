# 工控机TCP通讯

默认无密钥；Pi169.254.199.100，麒麟169.254.199.120。9010用于命令/状态/检测点，9002独立传HK，9030独立传RGB-D。保留SHA256、长度校验、去重及断线重连。

完整运动远程入口为robot_task_control的line_follow_arm_test.launch.py remote_control:=true；只测相机为camera_capture_remote.launch.py。二者已包含本节点，不重复启动。单独桥接其他已加载工作流时，可用 `ros2 launch eli_cs_robot_serial industrial_pc.launch.py`。默认start_hk_transfer=false，让末尾采集流程按需占用9002。

麒麟工具在host_tools/，通过项目tools/export_industrial_pc.py导出。先listen_robot.py，另一个终端start_task.py或start_capture.py；检测点send_points.py。当前无需industrial_pc.key。旧签名模式只在两端明确设置use_shared_key=true并指定key_file时使用。

照片保存为 `年.月.日.时.分/hk/` 和 `年.月.日.时.分/RGB/`，例如 `2026.10.30.20.29/`。HK放海康JPEG，RGB放彩色/深度PNG，文件名以相同的时分秒毫秒前缀配对；原始16位深度不转换为8位，参数和深度比例集中保存在隐藏索引.capture_index.json。传输内部仍校验既有V5.4清单，临时ZIP不重复长期保存。

命令固定桥接/task_controller/start、/stop、/capture及/line_follow/resume。采集与运动互斥，同ID请求持久去重。收到坐标只发布/industrial_pc/camera_points，演示点为/industrial_pc/demo_camera_points，不自动执行触碰。

详细启动步骤见host_tools/QUICKSTART.md，线缆字段和检测点示例见项目根目录TCP通讯协议_麒麟工控机.md。
