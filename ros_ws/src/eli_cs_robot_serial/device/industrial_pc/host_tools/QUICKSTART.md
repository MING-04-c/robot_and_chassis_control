# 两端启动和收图

当前版本默认无密钥。把新版整个工具目录复制给麒麟，两端程序一起更新，无需复制industrial_pc.key。

## 1. 树莓派更新代码并加载环境

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select eli_cs_robot_serial robot_task_control
source install/setup.bash
```

只测相机：

```bash
ros2 launch robot_task_control camera_capture_remote.launch.py
```

完整循迹/机械臂任务：

```bash
# 本次开机CAN尚未配置时先执行一次，launch不会自动执行。
sudo bash scripts/setup_can.sh auto
ros2 launch robot_task_control line_follow_arm_test.launch.py remote_control:=true
```

这两种入口只选一种，都包含9010通讯。不另开工业PC通讯、HK传输或相机预览程序。完整入口沿用原robot_ip等参数，运动前仍进行原有就绪检查。

## 2. 麒麟先监听

进入工具目录，终端一保持运行：

```bash
python3 listen_robot.py
```

默认Pi地址169.254.199.100，麒麟网口169.254.199.120。control=connected、capture_service=True表示采集接口已就绪；末尾发送前hk=waiting_sender正常。

终端二，单独拍照：

```bash
python3 start_capture.py
```

完整运动任务则用：

```bash
python3 start_task.py
```

独立拍照不执行运动；若运动任务正在执行会明确拒绝。收到camera capture accepted和run_id表示已受理，后续phase=final_capture_running，发送完成为capture_complete。网络重发同一条命令用 `--message-id 上次编号`，不会重复执行。

## 3. 找照片

主机：`工具目录/received/2026.10.30.20.29/`（年月日时分示例）。
树莓派：`~/robot_data/task_final/2026.10.30.20.29/`。

此目录直接分为 `hk/`、`RGB/`：海康为 `hk/20.29.12_345_hk.jpg`，深度相机为 `RGB/20.29.12_345_rgb.png` 和 `RGB/20.29.12_345_depth.png`。同一分钟多次采集共用两个目录，秒/毫秒前缀保持配对并避免覆盖。深度保留原始16位。参数、深度比例和去重记录集中放在根目录隐藏索引 `.capture_index.json`，不再每张图生成一堆文件。

树莓派会先等待机器人静止延时，再启动HK预热、检查画面稳定并采集，关闭HK后才启动深度相机，保存同一SDK FrameSet的RGB/原始深度和参数；关闭两台相机后依次发送。成功后清掉重复的临时ZIP、NPY和清单；失败保留 `.pending` 中的临时数据用于排查，已保存的照片不丢弃。

主机看到[hk]保存完成、[rgbd]保存完成才是本轮收图证据。深度回ACK后Pi任务完成；HK保持旧通道，尚无落盘ACK。

## 4. 排查

```bash
python3 industrial_pc_network_test.py interfaces
python3 industrial_pc_network_test.py connect --host 169.254.199.100 --ports 9010
python3 start_task.py --action status
```

- 连接失败：树莓派9010是否监听，IP/网线/端口是否正确；Pi可用 `ss -lntp | grep ':9010'` 查看。
- 提示密钥/模式不匹配：旧版文件混用，替换整个新版目录。默认无需密钥；config.json中use_shared_key=false。
- No camera found：SDK没有找到HK，检查相机连接和供电。
- 画面未稳定：检查曝光/运动情况，参数见Pi的final_capture.yaml。
- 不能监听9030：已有另一个监听程序占用，保持一个接收进程。
- 任务运行中：不能重复启动或并发独立采集。

途中站继续和停止沿用：

```bash
python3 start_task.py --action resume
python3 start_task.py --action stop
```

旧照片目录可用 `python3 capture_storage.py --source received --output received_clean` 复制整理，原数据不删除。检测点仍用 `python3 send_points.py --file result.json` 返回，格式和坐标单位沿用原协议。

## 相机点返回后真实探针触碰测试

与前述两个入口二选一：麒麟先 `python3 listen_robot.py`，Pi运行 `ros2 launch robot_task_control camera_detection_real.launch.py`。就绪后自动采新图并发送，等待主机检测结果；主机运行 `python3 send_points.py --file result.json`，匹配本次照片后真机会自动接近并限力触碰。不需要再发start_task或start_capture。

直接给YAML点测试用 `ros2 launch robot_task_control camera_detection_real.launch.py target_source:=local`，该模式不加载相机和TCP。两模式共用20N停止推进、保持5秒、逐点返回原位。图像与拍照位姿在等待期间保持对应，旧图或DEMO不启动触碰。
