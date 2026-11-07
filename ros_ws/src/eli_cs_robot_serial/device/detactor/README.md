# 漆膜测厚串口采集任务

接口保留原有拼写 `detactor`。默认串口 `/dev/ttyCH341USB0`，115200、8位、1停止位、无校验和流控。
发送普通ASCII文本 `.d0<:>`，不按十六进制文本发送、不添加换行。
接收原协议 `.e数值;数值;…<:>`；支持分包、粘包和小数。设备历史窗口末尾为最新数据，
发布单值取整帧最后一个数。滚动窗口 `/detactor/latest_values` 则按最新在前排列，最多10项。
不猜测二进制回包、倍率或单位；尚未确认的单位保留 `unknown`。

在 ros_ws 加载ROS环境后使用：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 独立连续采集，每秒查询一次。
ros2 launch eli_cs_robot_serial detactor.launch.py
# 或只启动串口，按任务请求采集。两条launch不要同时运行。
ros2 launch eli_cs_robot_serial detactor.launch.py auto_request:=false
# 或启动后自动完成一次采集，之后保持接口供后续任务调用。
ros2 launch eli_cs_robot_serial detactor.launch.py task_once:=true
```

串口不同可加 `device:=/dev/ttyUSB0`，波特率可用 `baudrate:=115200` 指定。
连接状态 `/detactor/connected` 表示串口已打开，不代表仪器已经回包。
串口打开失败自动重连；打开成功立即进入接收，不再额外等2秒。

在另一终端加载同样环境：

```bash
# 可调用的采集任务：等本次新有效回包才返回成功，默认3秒超时。
ros2 service call /detactor/acquire_latest std_srvs/srv/Trigger '{}'
# 只看最新一个数。
ros2 topic echo /detactor/latest_value std_msgs/msg/Float64 --qos-durability transient_local
# 带接收时间、序号、单位的最新单值JSON。
ros2 topic echo /detactor/measurement std_msgs/msg/String --qos-durability transient_local
# 本次采集任务的成功/失败结果。
ros2 topic echo /detactor/task_result std_msgs/msg/String --qos-durability transient_local
```

采集中再次调用会返回busy，防止命令重叠。`acquire_latest`不会返回上次缓存冒充本次采集；
同一数值的新回包仍更新序号和接收时间。接收时间表示树莓派收到回包的时间，不能证明仪器内部历史值已更新。
`/detactor/latest_measurement`只读缓存，旧 `/detactor/acquire`仍仅表示请求已排队。
缓存话题的保留值不保证当前连接有效；控制和逐点回传使用时间/批次关联的结果。
数据默认保存在 `~/.ros/eli_cs_robot_serial/detactor`，也可指定 `data_directory:=~/.ros/eli_cs_robot_serial/detactor`。

## 接入探针检测和麒麟回传

`camera_detection_real.launch.py`默认启动本串口且关闭周期采集。
20N停止并进入5秒保持时，向 `/detactor/acquire_trigger` 触发一次采集，
仅关联本次接触之后收到的有效测厚回包。撤离并确认实际关节回到记录原位、外力回落后，
`/detect_controller/point_result`发布逐点结果，协调器补充point_id和照片身份后发到
`/task_controller/detection_result`。
TCP桥接将结果落盘在配置output_dir下的 `detection_results/`，通过9010状态查询返回最近100条。
麒麟运行更新后的 `python3 listen_robot.py`，自动将结果保存到其output_dir下的
`detection_results/`；一个结果ID一份JSON，重复查询/监听重启不重复生成文件。
因此无需在机械臂回位后再读取一个可能来自其他任务的全局缓存。

如果保持期间没有有效回包，该点不能宣布测厚成功；回原位后进入重新拍照取点流程。
测厚仪自身仍使用串口；树莓派与麒麟之间的TCP通讯继续保留。

TCP传输驱动保留为可选调试 `transport:=tcp`，不作为当前仪器的运行方式。
