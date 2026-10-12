# MASC Camera TCP Protocol

This protocol sends ROS `/camera/image` frames from `masc_user_serial` over direct TCP. The camera computer listens on its Ethernet address `169.254.199.100:9002` and accepts only connections from `169.254.199.250`. The text/debug service on port 9001 is separate. The receiving computer does not need ROS 2.

## Choose the image mode

The combined launch starts the Hik camera driver with `show_image=false`, so no preview window consumes image-processing resources. JPEG mode is the default:

```bash
# JPEG mode (default; lossy, usually smaller)
ros2 launch masc_user_serial masc_tcp.launch.py image_format:=jpeg

# Raw mode (exact bytes of the ROS sensor_msgs/Image.data field)
ros2 launch masc_user_serial masc_tcp.launch.py image_format:=raw
```

`raw` does not JPEG encode or otherwise modify `Image.data`. It does **not** contain the sensor's original Bayer bytes if the camera driver already converted them: the current Hik driver publishes `rgb8` after SDK conversion. The raw TCP payload is exactly that ROS `rgb8` image's data, including any row padding.

Both modes send at most `image_max_fps` (default 60) frames per second. The server keeps only its latest frame, so slow clients and the frame-rate limit can skip camera frames. TCP preserves bytes within a delivered frame, but this protocol does not guarantee receipt of every camera frame.

## Receive on the other computer

Connect the computers by Ethernet. Camera computer: `169.254.199.100/24`. Receiving computer: `169.254.199.250/24`. Copy the **updated** `tools/receive_camera_tcp.py` to the receiving computer. Run the command matching the sender mode:

```bash
# Raw mode: exact pixel bytes and JSON metadata
python3 receive_camera_tcp.py --host 169.254.199.100 --port 9002 --latest latest.raw

# JPEG mode
python3 receive_camera_tcp.py --host 169.254.199.100 --port 9002 --latest latest.jpg
```

`latest.json` stores the matching metadata. `--count 1` receives one frame. `--save-dir frames` saves every received frame with its own JSON metadata. `--show` displays supported 8-bit images if OpenCV and NumPy are installed; saving needs only the Python standard library.

## Wire format

Each TCP connection is a sequence of frames. There is no request or authentication message. For each frame, read the following fields **exactly** in order; a single `recv()` may return fewer bytes than requested.

| Field | Size | Meaning |
|---|---:|---|
| Magic | 4 bytes | ASCII `MIMG` |
| Header length | 4 bytes | Unsigned 32-bit big-endian integer |
| Header | Header length bytes | UTF-8 JSON object |
| Payload | `payload_length` bytes | JPEG file (v1) or unchanged ROS Image.data bytes (v2) |

### Raw image, version 2

Example JSON header:

```json
{
  "version": 2,
  "type": "image",
  "sequence": 123,
  "stamp_ns": 1790246711679573373,
  "frame_id": "camera_optical_frame",
  "topic": "/camera/image",
  "width": 1440,
  "height": 1080,
  "step": 4320,
  "is_bigendian": 0,
  "encoding": "rgb8",
  "payload_length": 4665600
}
```

All integers are JSON numbers. `stamp_ns` is the original ROS image timestamp. `step` is bytes per row; `payload_length` must equal `step * height`. `is_bigendian` is the original ROS field (0 or 1). `encoding` is the original ROS image encoding, such as `rgb8`, `bgr8`, or `mono8`. There is no per-row or per-frame compression. The receiver can reconstruct a ROS `sensor_msgs/Image` with `height`, `width`, `encoding`, `is_bigendian`, `step`, and the payload as `data`. For `rgb8` with no padding, pixel `(x, y)` starts at byte offset `y * step + x * 3` and its bytes are R, G, B.

Python example for an `rgb8` image without row padding:

```python
import numpy as np
pixels = np.frombuffer(payload, dtype=np.uint8).reshape(header["height"], header["step"])
rgb = pixels[:, :header["width"] * 3].reshape(header["height"], header["width"], 3)
```

### JPEG image, version 1

The existing JPEG header remains compatible with earlier receivers:

```json
{
  "version": 1,
  "type": "image",
  "sequence": 123,
  "stamp_ns": 1790246711679573373,
  "frame_id": "camera_optical_frame",
  "topic": "/camera/image",
  "width": 1440,
  "height": 1080,
  "source_encoding": "rgb8",
  "encoding": "jpeg",
  "payload_length": 183421
}
```

The v1 payload is a complete JPEG file. JPEG is lossy. The updated receiver accepts both versions; a previous v1-only receiver will reject raw v2 frames.

`sequence` increments for frames emitted by the TCP sender during one process run. It can skip on a slow client. Reconnect at any time. The example receiver limits headers to 65,536 bytes, JPEG payloads to 32 MiB, and raw payloads to 128 MiB.

## Launch arguments

| Argument | Default | Meaning |
|---|---:|---|
| `image_stream` | `true` | Start image TCP service |
| `start_hik_camera` | `true` | Start Hik camera driver with its preview disabled |
| `image_topic` | `/camera/image` | ROS image topic |
| `image_listen_ip` | `169.254.199.100` | Camera computer Ethernet IP |
| `image_allowed_client_ip` | `169.254.199.250` | Only accepted client source IP |
| `image_port` | `9002` | Image TCP port |
| `image_format` | `jpeg` | `jpeg` or `raw` |
| `image_jpeg_quality` | `80` | JPEG quality, ignored in raw mode |
| `image_max_fps` | `60.0` | Maximum transmitted frame rate |

The server subscribes using ROS sensor-data QoS and builds frames only while a TCP client is connected.
