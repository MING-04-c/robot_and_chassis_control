# eli_cs_robot_serial

ROS 2 USB serial bridge. Received data is published as text and raw bytes. ROS
topics and a service can write ASCII data to the external serial device.

## Configure

Edit `config/serial.yaml` and set `device` and `baudrate`. With `line_mode: true`,
the external device must terminate each message with `\n`. Set it to `false`
for unframed binary data.

The default serial format is 8 data bits, no parity, one stop bit, and no flow
control (8N1).

## Build and launch

```bash
cd ~/project/ls65_ros/ls65/eli_ros_ws
colcon build --packages-select eli_cs_robot_serial --symlink-install
source install/setup.bash
ros2 launch eli_cs_robot_serial serial.launch.py
```

## Receive

```bash
ros2 topic echo /serial/rx
ros2 topic echo /serial/rx_bytes
```

## Send

All typed values are converted to ASCII. `append_newline: true` appends `\n`.

```bash
ros2 topic pub --once /serial/tx std_msgs/msg/String "{data: 'START'}"
ros2 topic pub --once /serial/tx_float std_msgs/msg/Float64 "{data: 3.14}"
ros2 topic pub --once /serial/tx_int std_msgs/msg/Int64 "{data: 42}"
ros2 service call /serial/write eli_cs_robot_serial/srv/WriteSerial "{data: 'RESET'}"
```

Send a file as raw binary data. File transfer does not append a newline:

```bash
ros2 service call /serial/send_file eli_cs_robot_serial/srv/SendFile \
  "{path: '/home/robot/data.bin'}"
```

## Multiple serial ports

Up to three independent serial-node instances can be started by one launch file.
An empty device argument disables that port.

```bash
ros2 launch eli_cs_robot_serial serial_multi.launch.py \
  port1_device:=/dev/ttyUSB0 port1_baudrate:=115200 \
  port2_device:=/dev/ttyUSB1 port2_baudrate:=9600 \
  port3_device:=/dev/ttyACM0 port3_baudrate:=115200
```

Each port uses its own namespace, for example `/serial1/rx`, `/serial2/tx_float`
and `/serial3/write`.

## TCP Ethernet

Edit `config/network.yaml`. In server mode this computer listens on `port`. In
client mode, `host` is the IPv4 address of the external device to connect to.

```bash
ros2 launch eli_cs_robot_serial tcp.launch.py
ros2 topic echo /tcp/rx
ros2 topic pub --once /tcp/tx std_msgs/msg/String "{data: 'START'}"
ros2 topic pub --once /tcp/tx_float std_msgs/msg/Float64 "{data: 3.14}"
ros2 topic pub --once /tcp/tx_int std_msgs/msg/Int64 "{data: 42}"
ros2 service call /tcp/write eli_cs_robot_serial/srv/WriteSerial "{data: 'RESET'}"
```

Send a raw binary file over TCP:

```bash
ros2 service call /tcp/send_file eli_cs_robot_serial/srv/SendFile \
  "{path: '/home/robot/data.bin'}"
```

The receiver must define its own file framing protocol, such as a header with
file size, because TCP and serial streams do not preserve message boundaries.
