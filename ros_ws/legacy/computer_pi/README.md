# Raspberry Pi TCP communication

The Raspberry Pi is expected to run a TCP server at `169.254.199.100:9001`.
This ROS 2 node runs as a TCP client and reconnects automatically after a
connection failure.

## Unified start

Start the status connection on port 9001, the camera receiver on port 9002,
the ROS image bridge, snapshot service, and live image window together:

```bash
cd ~/project/ls65_ros/ls65/eli_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch eli_cs_robot_serial computer_pi.launch.py
```

The image window is enabled by default. Use `show_image:=false` when running
without a desktop. `enable_status:=false` and `enable_camera:=false` can disable
either connection independently.

The status protocol is exposed as these topics:

- `/computer_pi/status`: latest task or connection status as JSON
- `/computer_pi/online`: connection availability as `std_msgs/msg/Bool`
- `/computer_pi/camera_info`: latest camera information message
- `/computer_pi/chassis_status`: chassis task status messages
- `/computer_pi/robot_status`: robot task status messages

Task states are `0=idle`, `1=running`, `2=completed`, `3=error`, and
`4=offline`. The initial state is offline. After a valid protocol message the
state becomes online; if no valid message is received for 3 seconds, the node
publishes state 4 again. The timeout can be changed with
`offline_timeout:=SECONDS`. The node sends a heartbeat every second and
automatically acknowledges completed and error events using their `event_id`.

## Start

```bash
cd ~/project/ls65_ros/ls65/eli_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch eli_cs_robot_serial computer_pi_tcp.launch.py
```

Override the endpoint without editing the YAML file:

```bash
ros2 launch eli_cs_robot_serial computer_pi_tcp.launch.py \
  host:=192.168.1.20 port:=9000
```

## Receive

```bash
ros2 topic echo /computer_pi/rx
ros2 topic echo /computer_pi/rx_bytes
```

`/computer_pi/rx` publishes one message for every LF-terminated text line.
`/computer_pi/rx_bytes` publishes the same payload as a byte array.

## Send

```bash
ros2 topic pub --once /computer_pi/tx \
  std_msgs/msg/String "{data: 'hello pi'}"

ros2 topic pub --once /computer_pi/tx_float \
  std_msgs/msg/Float64 "{data: 3.14}"

ros2 topic pub --once /computer_pi/tx_int \
  std_msgs/msg/Int64 "{data: 42}"

ros2 service call /computer_pi/write \
  eli_cs_robot_serial/srv/WriteSerial "{data: 'hello pi'}"
```

Text, float, and integer messages are sent as ASCII followed by LF when
`append_newline: true`.

Send a file as raw binary data:

```bash
ros2 service call /computer_pi/send_file \
  eli_cs_robot_serial/srv/SendFile \
  "{path: '/home/robot/data.bin'}"
```

TCP is a byte stream and does not preserve message boundaries. Both computers
must use the same text delimiter or application-level binary frame format.

## Camera stream (MASC Camera TCP Protocol v1/v2)

The camera stream is independent of the text/debug connection on port 9001.
The camera computer runs `hik_camera_ros2_driver` and
`ros2 launch masc_user_serial masc_tcp.launch.py`, listening on
`169.254.199.100:9002`. This computer must have `169.254.199.250/24`
on its Ethernet interface because the server accepts only that source IP.

```bash
ros2 launch eli_cs_robot_serial computer_pi_camera_tcp.launch.py
```

The launch command also opens a live camera window. It displays the measured
receive FPS, accumulated frame count, image size, and source encoding. Press
`q` or `Esc` to close the window. On a headless machine, use
`show_image:=false`.

Protocol v1 carries JPEG frames. Protocol v2 carries exact ROS Image bytes
using the header fields `encoding`, `width`, `height`, `step`, and
`is_bigendian`. The receiver supports raw `rgb8`, `bgr8`, `rgba8`, `bgra8`,
`mono8`, `mono16`/`16UC1`, and common 8-bit Bayer frames. It converts raw
pixels to JPEG locally, reconnects after a disconnect, and publishes the JPEG as
`sensor_msgs/msg/CompressedImage` on
`/computer_pi/camera/image/compressed`, preserving the source frame ID and
timestamp. The latest JPEG and its JSON header are atomically saved to
`/tmp/computer_pi_camera/latest.jpg` and `latest.json`. The files are
overwritten on each frame. Set `latest_file:=''` to disable file output.

## ROS image topic and snapshot service

The image bridge republishes each decoded frame as
`sensor_msgs/msg/Image` with `bgr8` encoding on
`/computer_pi/camera/image`. It only copies the full raw image while that
topic has subscribers. The JPEG stream remains available on
`/computer_pi/camera/image/compressed`.

Save the latest received frame by calling:

```bash
ros2 service call /computer_pi/camera/capture std_srvs/srv/Trigger "{}"
```

A successful response contains the saved JPEG path. Files are named with the
capture time and are written to
`/home/robot/Pictures/computer_pi_camera` by default. The image topic and
capture service remain active when the launch argument `show_image:=false`
disables the window.

The launch arguments `host`, `port`, `local_ip`, `compressed_topic`,
`image_topic`, `capture_service`, `snapshot_directory`, `latest_file`,
`reconnect_period`, `jpeg_quality`, and `show_image` can be overridden. For example:

```bash
ros2 launch eli_cs_robot_serial computer_pi_camera_tcp.launch.py \
  host:=169.254.199.100 port:=9002 local_ip:=169.254.199.250
```

For raw data, the sender must use protocol `version: 2` and put the ROS
image encoding directly in `encoding`, for example `"encoding": "rgb8"`.
The payload length must equal `step * height`. The receiver validates the MIMG
magic, protocol version, JSON header, dimensions, byte order, row stride,
payload size, and length limits (64 KiB header, 128 MiB raw image). It reads
each field to completion because TCP does not preserve frame boundaries.

## Chassis arrival and control

This computer acts as the supervisor on the status connection. It receives the
chassis status and sends control commands to the chassis side.

Watch the complete chassis status or only its arrival flag:

```bash
ros2 topic echo /computer_pi/chassis_status
ros2 topic echo /computer_pi/chassis_arrived
```

`/computer_pi/chassis_arrived` is true only when the latest chassis status is
`completed`. It changes to false for other states and when the peer is offline.
Completed and error chassis states are acknowledged automatically using their
`event_id`.

Send chassis commands:

```bash
# Continue
ros2 service call /computer_pi/chassis/control \
  eli_cs_robot_serial/srv/ChassisControl "{command: 0, task: ''}"

# Stop
ros2 service call /computer_pi/chassis/control \
  eli_cs_robot_serial/srv/ChassisControl "{command: 1, task: ''}"

# Execute a task
ros2 service call /computer_pi/chassis/control \
  eli_cs_robot_serial/srv/ChassisControl "{command: 2, task: 'forward_10cm'}"
```

Command values are `0=continue`, `1=stop`, and `2=execute_task`. Calls fail while
the status connection is offline. Unacknowledged commands are resent every 500 ms.

The control command is an extension to protocol v1 and is sent as one NDJSON
line. The server on `169.254.199.100:9001` must handle this layout:

```json
{"version":1,"type":"control_command","sequence":20,"timestamp_ns":1790593050000000000,"source":"supervisor","event_id":"control-abc123","command":"continue"}
{"version":1,"type":"control_command","sequence":21,"timestamp_ns":1790593051000000000,"source":"supervisor","event_id":"control-def456","command":"stop"}
{"version":1,"type":"control_command","sequence":22,"timestamp_ns":1790593052000000000,"source":"supervisor","event_id":"control-ghi789","command":"execute_task","task":"forward_10cm"}
```

The complete peer implementation contract is in `COMMUNICATION_PROTOCOL.md`.
