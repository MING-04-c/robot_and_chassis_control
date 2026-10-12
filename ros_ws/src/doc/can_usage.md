# MASC 底盘 CAN 使用说明

本文说明如何在 Linux 上把 USB-CAN 设备配置成 MASC 底盘 CAN 软件可用的 SocketCAN 接口。

`masc_chassis_can_sdk` 和 `masc_chassis_ros_driver` 只打开 Linux SocketCAN 接口，例如
`can0`。SDK 不会自动执行 `ip link`、`slcand` 或设置 bitrate；启动应用前，需要先让
系统中存在一个已经 `UP` 的 CAN 网口。

默认参数：

```yaml
can:
  interface_name: can0
  bitrate: 500000
  expected_protocol_version: 10
```

当前 MASC 底盘协议使用 classic CAN 2.0A、11-bit standard ID、DLC `0-8`。即使 USB-CAN
硬件支持 CAN FD，运行本软件时也按 classic CAN 配置。

## 准备工具

Ubuntu 上安装 `can-utils`：

```bash
sudo apt update
sudo apt install -y can-utils
```

`candump`、`cansend` 等 CAN 测试命令由 `can-utils` 提供。

常用检查命令：

```bash
dmesg -w
lsusb
ls -l /dev/serial/by-id/
ip -details link show type can
```

接线检查：

* `CANH` 接 `CANH`，`CANL` 接 `CANL`。
* 建议共地，USB-CAN 的 `GND` 接到底盘 CAN 总线参考地。
* 总线两端各一个 `120 ohm` 终端电阻；不要在总线上重复打开过多终端。
* bitrate 必须和底盘固件一致，本仓库默认使用 `500000`。

## 快速配置脚本

仓库提供临时快速配置脚本：

```bash
cd ~/masc_ros_ws/src/masc_chassis_can_driver
# ROS1 工作区通常为：cd ~/catkin_ws/src/masc_chassis_can_driver
sudo ./scripts/setup_can.sh auto
```

`auto` 会优先使用系统中已经存在的原生 SocketCAN 网口，例如 `can0`。如果当前没有 CAN 网口，
脚本会查找 `/dev/serial/by-id/`、`/dev/ttyACM*`、`/dev/ttyUSB*` 中的 SLCAN 串口候选。
只有候选唯一时才会自动选择；如果发现多个候选，脚本会打印列表并要求手动指定，避免误配置
客户电脑上的其他串口设备。

常用命令：

```bash
# 自动识别并配置，默认 can0 / 500000
sudo ./scripts/setup_can.sh auto

# 原生 SocketCAN，例如 gs_usb、Candlelight、板载 CAN
sudo ./scripts/setup_can.sh socketcan --iface can0 --bitrate 500000

# CANable / SLCAN 串口模式
sudo ./scripts/setup_can.sh slcan --device /dev/ttyACM0 --iface can0 --bitrate 500000

# 推荐使用稳定 by-id 路径
sudo ./scripts/setup_can.sh slcan \
  --device /dev/serial/by-id/usb-Openlight_Labs_CANable2_...-if00 \
  --iface can0 \
  --bitrate 500000

# 查看 CAN 接口状态
./scripts/setup_can.sh status --iface can0

# 监听 system_version_feedback (0x205)
./scripts/setup_can.sh test --iface can0

# 拉下接口，并停止匹配 can0 的 slcand 进程
sudo ./scripts/setup_can.sh down --iface can0
```

脚本配置完成后会默认执行一次 `candump -tz can0,205:7FF` 检查。如果底盘未上电或总线没有
数据，脚本会提示没有收到 `0x205`，但这不一定表示 CAN 网口配置失败。只想配置接口、不等待
底盘反馈时可以加 `--skip-test`：

```bash
sudo ./scripts/setup_can.sh auto --skip-test
```

部分 USB-CAN 设备不支持 `restart-ms`，例如某些 `gs_usb` 设备可能提示
`Device doesn't support restart from Bus Off`。脚本会自动降级为只设置 bitrate；只要
`ip -details link show can0` 中能看到 `state ERROR-ACTIVE`、`bitrate 500000`，并且 RX
计数增加，说明 CAN 网口已经正常工作。

快速脚本只用于当前会话配置和现场排查，不负责开机自启动。需要开机自动创建 `can0` 时，优先
使用后面的 `can_service.sh` 自启动安装脚本。

## 路径一：CANable / CANable 2.0 的 SLCAN 模式

CANable 2.0 默认常见表现是 CDC ACM 串口设备。插入后 `dmesg` 类似：

```text
Product: CANable2 ...
Manufacturer: Openlight Labs
cdc_acm ... ttyACM0: USB ACM device
```

这表示 USB 已识别成功，但系统还不会自动出现 `can0`。需要用 `slcand` 把串口桥接成
SocketCAN 网口。

也可以用 USB VID/PID 辅助确认 CANable 2.0：

```bash
lsusb -d 16d0:117e
```

以 `/dev/ttyACM0`、`500 kbit/s`、接口名 `can0` 为例：

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan

sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ip link set can0 up

ip -details link show can0
```

更推荐使用稳定的 `by-id` 路径，避免插拔后 `ttyACM0` 变成其他编号：

```bash
ls -l /dev/serial/by-id/
sudo slcand -o -c -s6 /dev/serial/by-id/usb-Openlight_Labs_CANable2_...-if00 can0
sudo ip link set can0 up
```

`slcand -s` 常用 bitrate 对照：

| 参数 | bitrate |
|---|---:|
| `-s4` | `125000` |
| `-s5` | `250000` |
| `-s6` | `500000` |
| `-s7` | `800000` |
| `-s8` | `1000000` |

如果需要重新设置 bitrate，先停掉旧的 `slcand`，再用新的 `-s` 参数启动：

```bash
sudo ip link set can0 down
sudo pkill -f 'slcand.*ttyACM0'
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ip link set can0 up
```

如果没有权限访问 `/dev/ttyACM0`，可以先继续使用 `sudo`。需要普通用户访问时，把当前用户
加入 `dialout` 组，然后重新登录：

```bash
sudo usermod -aG dialout $USER
```

## 路径二：Candlelight / gs_usb 原生 SocketCAN 模式

Candlelight 固件走 Linux 内核 `gs_usb` 驱动。设备插入后通常会直接出现 `can0`、`can1`
这类 CAN 网口，不需要 `slcand`。

检查并加载驱动：

```bash
sudo modprobe gs_usb
dmesg | grep -i -E 'can|gs_usb|candle'
ip link show type can
```

以 `can0`、`500 kbit/s` 为例配置：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up

ip -details link show can0
```

`restart-ms 100` 会让 Linux CAN 控制器在 bus-off 后尝试自动恢复。SDK 仍只记录 transport
错误，是否重连由上层应用决定。

## CAN 总线验证

确认 `can0` 已经存在并处于 `UP`：

```bash
ip -details -statistics link show can0
```

监听总线：

```bash
candump -tz can0
```

只看 MASC 底盘协议版本反馈帧：

```bash
candump -tz can0,205:7FF
```

SDK `Connect()` 会等待 `system_version_feedback (0x205)`，并检查协议版本是否为 V10
也就是 `0x0A`。如果 `candump` 能看到底盘反馈，但上层应用仍连不上，优先检查
CAN 接口名、期望协议版本和底盘固件版本。

不要在真实底盘总线上随意 `cansend` 未确认含义的 CAN ID。需要测试发送能力时，建议在隔离
测试总线或假负载上进行。

完成 CAN 总线验证后，应用层使用说明见仓库根目录 `README.md`。

## 自启动配置

本节只说明 CAN 接口自启动。上层应用自启动请在确认 `can0` 准备好后另行配置。

### 推荐：自启动安装脚本

仓库提供自启动安装脚本：

```bash
cd ~/masc_ros_ws/src/masc_chassis_can_driver
# ROS1 工作区通常为：cd ~/catkin_ws/src/masc_chassis_can_driver
sudo ./scripts/can_service.sh install
```

`install` 不带模式时会自动识别并推荐配置方式：

* 已存在原生 SocketCAN 网口时，推荐 `socketcan`。
* 没有 CAN 网口但只有一个 SLCAN 串口候选时，推荐 `slcan`。
* 多个 CAN 网口或多个串口候选时，不自动安装，要求指定 `--iface` 或 `--device`。

脚本会写入：

```text
/etc/masc/can/can0.env
/etc/systemd/system/masc-can-can0.service
```

安装前会打印推荐配置并要求确认。非交互部署可以加 `--yes`，只预览不写入系统可以加
`--dry-run`：

```bash
# 自动推荐，打印配置并确认后安装
sudo ./scripts/can_service.sh install

# 非交互安装
sudo ./scripts/can_service.sh install --yes

# 只预览 env 和 service，不写入 /etc
./scripts/can_service.sh install --dry-run

# 显式安装原生 SocketCAN 自启动
sudo ./scripts/can_service.sh install socketcan --iface can0 --bitrate 500000

# 显式安装 CANable / SLCAN 自启动，推荐使用稳定 by-id 路径
sudo ./scripts/can_service.sh install slcan \
  --device /dev/serial/by-id/usb-Openlight_Labs_CANable2_...-if00 \
  --iface can0 \
  --bitrate 500000
```

管理命令：

```bash
sudo ./scripts/can_service.sh restart --iface can0
sudo ./scripts/can_service.sh uninstall --iface can0
./scripts/can_service.sh status --iface can0
./scripts/can_service.sh show --iface can0
```

如果希望安装后立即检查底盘版本反馈帧，可以加 `--test`：

```bash
sudo ./scripts/can_service.sh install --test
```

### 高级：手动自启动配置

下面的手动配置以 `can0`、`500000` bitrate 为例。原生 CAN 网口可以优先用
`systemd-networkd` 按普通网络接口方式管理；CANable/SLCAN 串口模式需要先用 `slcand`
创建 SocketCAN 网口。不要同时启用多个都会配置 `can0` 的方案。

### systemd-networkd 管理原生 CAN 网口

适用于系统启动后已经能看到 `can0` 的原生 CAN 网口，例如 SPI CAN 控制器 `mcp251x`，或
Candlelight/gs_usb 生成的 SocketCAN 网口。部分 Ubuntu 系统的 NetworkManager 会把
`can0` 显示成 `unmanaged`，而且 `nmcli` 版本不一定支持 `type can`；这种情况下可以让
`systemd-networkd` 只管理 `can0`，Wi-Fi 和以太网仍交给 NetworkManager。

创建 `/etc/systemd/network/80-can0.network`：

```ini
[Match]
Name=can0

[Link]
RequiredForOnline=no

[CAN]
BitRate=500K
RestartSec=100ms
```

启用并应用配置：

```bash
sudo systemctl enable --now systemd-networkd
sudo networkctl reload
sudo networkctl reconfigure can0
```

验证：

```bash
networkctl status can0
ip -details link show can0
candump can0
```

`ip -details link show can0` 中应能看到 `state ERROR-ACTIVE` 或正常 CAN 状态、
`bitrate 500000`、`restart-ms 100`。如果 CAN 接口不是 `can0`，把 `[Match]` 中的
`Name=can0` 改成实际接口名，并同步修改上层应用里的 CAN 接口名配置。

SLCAN 模式下系统启动时通常只有 `/dev/ttyACM0`，还没有 `can0`，因此不能只靠
`systemd-networkd` 生成接口，需要使用下面的 `slcand` service。

### CANable / SLCAN 自启动

先确认稳定设备路径：

```bash
ls -l /dev/serial/by-id/
```

例如 CANable 2.0 可能显示：

```text
/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_206534AB3845-if00
```

创建 `/etc/systemd/system/canable-can0.service`：

```ini
[Unit]
Description=CANable SLCAN bridge for can0
After=systemd-udevd.service

[Service]
Type=simple
ExecStartPre=/usr/sbin/modprobe can
ExecStartPre=/usr/sbin/modprobe can_raw
ExecStartPre=/usr/sbin/modprobe slcan
ExecStart=/usr/bin/slcand -F -o -c -s6 /dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_206534AB3845-if00 can0
ExecStartPost=/usr/bin/bash -lc 'for i in {1..30}; do /usr/sbin/ip link show can0 >/dev/null 2>&1 && exit 0; sleep 0.2; done; exit 1'
ExecStartPost=/usr/sbin/ip link set can0 up
ExecStopPost=-/usr/sbin/ip link set can0 down
Restart=on-failure
RestartSec=2
StartLimitIntervalSec=0

[Install]
WantedBy=multi-user.target
```

把 `ExecStart` 中的 `/dev/serial/by-id/...` 替换成自己机器上的 CANable 路径。如果底盘不是
`500000` bitrate，把 `-s6` 改成对应的 `slcand -s` 参数。

启用并立即启动：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now canable-can0.service
```

查看状态和日志：

```bash
systemctl status canable-can0.service
journalctl -u canable-can0.service -f
ip -details link show can0
candump can0
```

`candump can0` 能看到持续刷新的 CAN 帧，才说明 CAN 网口已经 `UP` 且总线上确实有数据。
如果命令没有输出，按后面的“`candump` 没有任何帧”排查。

停止并禁止自启动：

```bash
sudo systemctl disable --now canable-can0.service
```

### Candlelight / gs_usb 自启动

如果设备刷的是 Candlelight 固件，Linux 会通过 `gs_usb` 生成原生 SocketCAN 网口。创建
`/etc/systemd/system/can0-candlelight.service`：

```ini
[Unit]
Description=Configure candleLight gs_usb SocketCAN can0
After=systemd-udevd.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/usr/sbin/modprobe gs_usb
ExecStartPre=/usr/bin/bash -lc 'for i in {1..30}; do /usr/sbin/ip link show can0 >/dev/null 2>&1 && exit 0; sleep 0.2; done; exit 1'
ExecStart=/usr/sbin/ip link set can0 down
ExecStart=/usr/sbin/ip link set can0 type can bitrate 500000 restart-ms 100
ExecStart=/usr/sbin/ip link set can0 up
ExecStop=-/usr/sbin/ip link set can0 down
Restart=on-failure
RestartSec=2
StartLimitIntervalSec=0

[Install]
WantedBy=multi-user.target
```

启用并立即启动：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now can0-candlelight.service
```

验证：

```bash
systemctl status can0-candlelight.service
journalctl -u can0-candlelight.service -f
ip -details link show can0
candump can0
```

`candump can0` 能看到持续刷新的 CAN 帧，才说明 CAN 网口已经 `UP` 且总线上确实有数据。
如果命令没有输出，按后面的“`candump` 没有任何帧”排查。

如果系统里原生 CAN 网口不是 `can0`，把 service 中所有 `can0` 改成实际接口名，并同步修改
上层应用里的 CAN 接口名配置。

## 常见问题

### 只有 `/dev/ttyACM0`，没有 `can0`

这是 CANable/CANable 2.0 的 SLCAN 模式正常现象。执行 `slcand` 后才会生成 SocketCAN
接口。Candlelight/gs_usb 模式才会插上后直接生成 `can0`。

### `ioctl(SIOCGIFINDEX) failed` 或应用提示找不到接口

应用打开的是配置项中指定的 SocketCAN 网口。先用下面命令确认接口存在：

```bash
ip link show type can
```

如果没有接口，按 CANable/SLCAN 或 Candlelight/gs_usb 对应章节先配置硬件。

### `RTNETLINK answers: Device or resource busy`

通常是接口已经 `UP`，或已有 `slcand` 占用了同一个串口。先执行：

```bash
sudo ip link set can0 down
```

SLCAN 模式下需要重启 `slcand` 才能改变 bitrate。

### `candump` 没有任何帧

按顺序检查：

* bitrate 是否和底盘一致。
* CANH/CANL 是否接反。
* 底盘是否上电，CAN 收发器是否处于正常模式。
* 终端电阻是否正确。
* `ip -details -statistics link show can0` 是否出现 `bus-off`、`error-passive` 或错误计数增长。

### 应用已启动但一直未连接

先确认 `candump -tz can0,205:7FF` 能看到 `system_version_feedback (0x205)`。如果能看到帧，
再检查：

* 应用配置的 CAN 接口名是否是当前 CAN 网口名。
* 期望协议版本是否和底盘固件一致，默认 V10 为 `10`。
* SDK 或上层应用记录的连接失败原因。
* CAN ID `0x205` 的 DLC 和 payload 是否符合当前协议。

## 参考

* CANable Getting Started: <https://canable.io/getting-started.html>
* CANable 2.0: <https://openlightlabs.com/products/canable-2-0>
* Linux SocketCAN: <https://docs.kernel.org/networking/can.html>
* candleLight firmware: <https://github.com/candle-usb/candleLight_fw>
