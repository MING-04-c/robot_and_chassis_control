# Gemini Pro 相机驱动（ROS 2 Jazzy）

适用环境：当前已运行成功的树莓派 5B、Ubuntu 24.04 ARM64、ROS 2 Jazzy。
使用 Orbbec SDK v1.10.35，不要替换为 SDK v2。

## 放在哪里

将整个 `gemini_pro_ros2` 文件夹复制到目标工作空间的 `src` 内，例如：

```text
~/robot_ws/
└── src/
    └── gemini_pro_ros2/
        ├── orbbec_camera/
        ├── orbbec_camera_msgs/
        ├── orbbec_description/
        ├── LICENSE
        ├── NOTICE
        └── README.md
```

这三个包都保留原来的 ROS 包名。目标工作空间内不要另放同名包。
不要复制原工作空间的 build、install、log；也不需要 ros2_astra_camera-master。
请使用纯英文且不含空格的目标工作空间路径，之前中文路径下的消息生成失败。

## 编译

以下假设目标工作空间为 `~/robot_ws`，实际使用时替换这个路径。
在这台电脑上，驱动依赖此前已安装，可以先直接编译。

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-up-to orbbec_camera orbbec_description \
  --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DUSE_NV_HW_DECODER=OFF -DUSE_RK_HW_DECODER=OFF
```

## 启动

编译成功后，在同一个终端执行：

```bash
source install/setup.bash
ros2 launch orbbec_camera gemini_pro.launch.py
```

保持终端运行，按 Ctrl+C 停止。启动文件自动加载包内的
`config/gemini_pro_jazzy.yaml`，不依赖原下载目录或原工作空间。
默认启用彩色和深度，关闭红外、点云及深度对齐，沿用之前跑通的配置。
如需修改参数，编辑目标 src 内的 YAML 并重新编译该包，然后重启节点。

## 查看图像

另开终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
rviz2
```

在 RViz 中 Add → Image，Topic 选 `/color/image_raw`（彩色）
或 `/depth/image_raw`（深度）。Transport Hint 选 raw。
只启动驱动不会自动打开图像窗口。

## USB 权限

同一台电脑无需再次配置。换电脑时需安装依赖并设置 USB 权限；
在目标工作空间根目录执行以下命令，再拔插相机：

```bash
sudo install -m 644 \
  src/gemini_pro_ros2/orbbec_camera/scripts/99-obsensor-libusb.rules \
  /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 验证范围

原驱动和这份参数已在当前电脑成功启动，用户已查看图像。
整理版增加了标准 ROS 2 launch 文件；搬到目标工作空间后需重新编译验证。
