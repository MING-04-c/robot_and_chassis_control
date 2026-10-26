# MASC 底盘 CAN ROS 软件包

多机协同（苏州）有限公司（简称MASC）。该目录提供 MASC 底盘 CAN 通讯的软件包集合，
包含 CAN 协议 SDK、ROS 消息接口和 ROS 驱动节点。

当前默认协议版本：`10`

## 软件包

* `masc_chassis_can_sdk`: 纯 C++ CAN 协议 SDK，负责协议编解码、SocketCAN 通讯和状态缓存。
* `masc_chassis_msgs`: MASC 底盘 ROS 消息和服务接口。
* `masc_chassis_ros_driver`: 基于 SDK 的 ROS1/ROS2 底盘驱动节点。

## 特性

* 支持 Linux SocketCAN，例如 `can0`。
* 支持 ROS1 和 ROS2 双构建。
* 支持 `cmd_vel` 速度控制，命令超时后自动发送零目标。
* 发布 odom、底盘状态、BMS、电机、舵机、IMU、遥控器、通讯状态和 CAN 帧统计。
* 提供运动模式切换、拖车模式、清故障、舵机自动标定服务；运动模式切换仅当前定义的全轮转向/舵轮底盘支持。
* SDK callback 进入 ROS driver 时立即打 ROS 时间戳，便于上层判断数据时效。

## CAN 接口设置

CANable、CANable 2.0、Candlelight/gs_usb 的 Linux 配置步骤见
[`docs/can_usage.md`](docs/can_usage.md)。

以 `can0`、`500000` bitrate 为例：

```bash
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

如果使用 USB-CAN 或其他 CAN 设备，请先确认系统中的接口名：

```bash
ip link
```

## ROS2 软件包安装

以下命令使用 `$ROS_DISTRO` 表示本机 ROS2 发行版。如果当前终端未设置该变量，请先按实际
ROS2 版本设置，例如 `export ROS_DISTRO=<your_ros2_distro>`。

1. 安装 ROS 依赖包。

    ```bash
    sudo apt-get install -y \
      ros-$ROS_DISTRO-geometry-msgs \
      ros-$ROS_DISTRO-nav-msgs \
      ros-$ROS_DISTRO-sensor-msgs \
      ros-$ROS_DISTRO-std-msgs \
      ros-$ROS_DISTRO-std-srvs \
      ros-$ROS_DISTRO-tf2 \
      ros-$ROS_DISTRO-tf2-geometry-msgs \
      ros-$ROS_DISTRO-tf2-ros
    ```

2. 创建工作区并获取驱动源码。

    ```bash
    mkdir -p ~/masc_ros_ws/src
    cd ~/masc_ros_ws/src
    git clone https://github.com/masc-robotics/masc_chassis_can_driver.git masc_chassis_can_driver
    ```

3. 编译 MASC 底盘 CAN 软件包。

    ```bash
    cd ~/masc_ros_ws
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --base-paths src --packages-select \
      masc_chassis_can_sdk \
      masc_chassis_msgs \
      masc_chassis_ros_driver
    source install/setup.bash
    ```

4. 工作空间环境 source，可按需要写入 `~/.bashrc`。

    ```bash
    source ~/masc_ros_ws/install/setup.bash
    ```

## ROS1 软件包安装

以下命令使用 `$ROS_DISTRO` 表示本机 ROS1 发行版。如果当前终端未设置该变量，请先按实际
ROS1 版本设置，例如 `export ROS_DISTRO=<your_ros1_distro>`。

创建工作区、获取驱动源码并编译：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/masc-robotics/masc_chassis_can_driver.git masc_chassis_can_driver
cd ~/catkin_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make --pkg masc_chassis_can_sdk masc_chassis_msgs masc_chassis_ros_driver
source devel/setup.bash
```

## ROS Package 基本用法

### ROS2 单独启动底盘驱动

```bash
ros2 launch masc_chassis_ros_driver masc_chassis_ros_driver.launch.py
```

指定参数文件：

```bash
ros2 launch masc_chassis_ros_driver masc_chassis_ros_driver.launch.py \
  driver_params:=$HOME/masc_ros_ws/src/masc_chassis_can_driver/masc_chassis_ros_driver/config/ros2/driver_params.yaml
```

默认参数文件：

```text
masc_chassis_ros_driver/config/ros2/driver_params.yaml
```

### ROS1 单独启动底盘驱动

```bash
roslaunch masc_chassis_ros_driver masc_chassis_ros_driver_ros1.launch
```

指定参数文件：

```bash
roslaunch masc_chassis_ros_driver masc_chassis_ros_driver_ros1.launch \
  driver_params:=$HOME/catkin_ws/src/masc_chassis_can_driver/masc_chassis_ros_driver/config/ros1/driver_params.yaml
```

默认参数文件：

```text
masc_chassis_ros_driver/config/ros1/driver_params.yaml
```

更多话题、服务和参数配置请参考
[`masc_chassis_ros_driver/README.md`](masc_chassis_ros_driver/README.md)。

## 使用测试

首先启动底盘驱动包，然后发布 `Twist` 消息控制底盘运动。

ROS2：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

ROS1：

```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

ROS 通讯状态、CAN 帧统计和底盘状态检查见
[`docs/ros_comm_check.md`](docs/ros_comm_check.md)。

## 安全注意事项

* 发送速度命令前请确认急停、遥控接管和周围环境安全。
* 首次测试建议使用较小速度，例如 `linear.x = 0.1`。
* 发布错误的 `Twist` 速度消息可能导致机器人快速运动，请随时准备使用急停或遥控器接管。
* 如果 `cmd_vel` 停止发布，驱动会在超时后发送零目标；默认超时时间为 `100 ms`。

## 常见问题

### 启动后一直未连接

检查：

* `can.interface_name` 是否和系统 CAN 接口一致。
* CAN 网口是否已经 `up`。
* bitrate 是否和底盘固件一致。
* CANH/CANL 是否接反，终端电阻是否正确。
* `comm_status.fail_reason` 是否为 `PROTOCOL_VERSION_MISMATCH`。

### 能连接但没有 odom

`odom` 只在收到未超时的 `MotionFeedback` 后发布。检查：

* `/masc_chassis/can_frame_stats` 中 motion feedback CAN ID 是否有频率。
* `freshness.motion_feedback_timeout_ms` 是否过小。
* 底盘固件是否启用了运动反馈发送。
