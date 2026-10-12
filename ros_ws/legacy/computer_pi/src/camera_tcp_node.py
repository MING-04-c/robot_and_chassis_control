#!/usr/bin/env python3
"""Receive MASC Camera TCP Protocol v1 and publish JPEG frames in ROS 2."""

import json
import os
import socket
import struct
import tempfile
import time

import cv2
import numpy as np

MAX_HEADER_LENGTH = 65_536
MAX_JPEG_LENGTH = 32 * 1024 * 1024
MAX_RAW_PAYLOAD_LENGTH = 128 * 1024 * 1024
RAW_ENCODINGS = {
    "rgb8", "bgr8", "rgba8", "bgra8", "mono8", "mono16", "16uc1",
    "bayer_rggb8", "bayer_bggr8", "bayer_gbrg8", "bayer_grbg8",
}


class CameraProtocolError(Exception):
    pass


def read_exact(connection, length, should_continue=lambda: True):
    data = bytearray()
    while len(data) < length:
        try:
            chunk = connection.recv(min(length - len(data), 64 * 1024))
        except socket.timeout:
            if not should_continue():
                raise InterruptedError("ROS is shutting down")
            continue
        if not chunk:
            raise EOFError("camera TCP connection closed")
        data.extend(chunk)
    return bytes(data)


def read_frame(connection, should_continue=lambda: True):
    if read_exact(connection, 4, should_continue) != b"MIMG":
        raise CameraProtocolError("invalid frame magic")
    header_length = struct.unpack("!I", read_exact(connection, 4, should_continue))[0]
    if not 0 < header_length <= MAX_HEADER_LENGTH:
        raise CameraProtocolError(f"invalid header length: {header_length}")
    try:
        header = json.loads(read_exact(connection, header_length, should_continue).decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise CameraProtocolError("invalid JSON header") from exc
    if not isinstance(header, dict) or header.get("type") != "image":
        raise CameraProtocolError("unsupported message type")
    version = header.get("version")
    encoding = str(header.get("encoding", "")).lower()
    length = header.get("payload_length")
    if version == 1 and encoding in {"jpeg", "jpg"}:
        if type(length) is not int or not 0 < length <= MAX_JPEG_LENGTH:
            raise CameraProtocolError(f"invalid JPEG length: {length}")
    elif version == 2:
        width = header.get("width")
        height = header.get("height")
        step = header.get("step")
        if not all(type(value) is int and value > 0 for value in (width, height, step)):
            raise CameraProtocolError("invalid raw image dimensions")
        if header.get("is_bigendian") not in (0, 1):
            raise CameraProtocolError("invalid raw image byte order")
        if encoding not in RAW_ENCODINGS:
            raise CameraProtocolError(f"unsupported raw image encoding: {encoding}")
        if (
            type(length) is not int
            or length != step * height
            or length > MAX_RAW_PAYLOAD_LENGTH
        ):
            raise CameraProtocolError(f"invalid raw image length: {length}")
    else:
        raise CameraProtocolError("unsupported image protocol version or encoding")
    stamp_ns = header.get("stamp_ns")
    if type(stamp_ns) is not int or stamp_ns < 0:
        raise CameraProtocolError("invalid image timestamp")
    if not isinstance(header.get("frame_id"), str):
        raise CameraProtocolError("invalid frame_id")
    payload = read_exact(connection, length, should_continue)
    if encoding in {"jpeg", "jpg"}:
        if not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
            raise CameraProtocolError("invalid JPEG payload")
    return header, payload


def raw_payload_to_bgr(header, payload):
    width = header.get("width")
    height = header.get("height")
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise CameraProtocolError("invalid raw image dimensions")
    encoding = str(header.get("encoding", "")).lower()
    if encoding == "raw":
        encoding = str(header.get("source_encoding", "")).lower()
    bytes_per_pixel = {
        "mono8": 1, "mono16": 2, "16uc1": 2,
        "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4,
        "bayer_rggb8": 1, "bayer_bggr8": 1,
        "bayer_gbrg8": 1, "bayer_grbg8": 1,
    }.get(encoding)
    if bytes_per_pixel is None:
        raise CameraProtocolError(f"unsupported raw encoding: {encoding}")
    minimum_step = width * bytes_per_pixel
    step = header.get("step", minimum_step)
    if type(step) is not int or step < minimum_step:
        raise CameraProtocolError(f"invalid raw image step: {step}")
    expected_length = step * height
    if len(payload) != expected_length:
        raise CameraProtocolError(
            f"raw payload size mismatch: expected {expected_length}, got {len(payload)}"
        )
    rows = np.frombuffer(payload, dtype=np.uint8).reshape(height, step)
    packed = np.ascontiguousarray(rows[:, :minimum_step])
    if encoding in {"mono16", "16uc1"}:
        is_bigendian = header.get("is_bigendian")
        if is_bigendian is None:
            byte_order = str(header.get("byte_order", "little")).lower()
            is_bigendian = byte_order in {"big", "be", "big_endian"}
        dtype = ">u2" if is_bigendian else "<u2"
        mono16 = packed.reshape(height, width, 2).view(dtype).reshape(height, width)
        return cv2.convertScaleAbs(mono16, alpha=255.0 / 65535.0)
    if bytes_per_pixel > 1:
        packed = packed.reshape(height, width, bytes_per_pixel)
    else:
        packed = packed.reshape(height, width)
    conversions = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
        "bayer_rggb8": cv2.COLOR_BAYER_RG2BGR,
        "bayer_bggr8": cv2.COLOR_BAYER_BG2BGR,
        "bayer_gbrg8": cv2.COLOR_BAYER_GB2BGR,
        "bayer_grbg8": cv2.COLOR_BAYER_GR2BGR,
    }
    if encoding in {"bgr8", "mono8"}:
        return packed
    return cv2.cvtColor(packed, conversions[encoding])


def payload_to_jpeg(header, payload, jpeg_quality):
    encoding = str(header.get("encoding", "")).lower()
    if encoding in {"jpeg", "jpg"}:
        return payload
    image = raw_payload_to_bgr(header, payload)
    success, encoded = cv2.imencode(
        ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
    )
    if not success:
        raise CameraProtocolError("failed to encode raw image as JPEG")
    return encoded.tobytes()


def atomic_write(path, data):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".camera-", dir=directory)
    try:
        with os.fdopen(fd, "wb") as output:
            output.write(data)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = Node("computer_pi_camera_tcp_node")
    try:
        host = node.declare_parameter("host", "169.254.199.100").value
        port = node.declare_parameter("port", 9002).value
        local_ip = node.declare_parameter("local_ip", "169.254.199.250").value
        topic = node.declare_parameter(
            "compressed_topic", "/computer_pi/camera/image/compressed"
        ).value
        latest_file = node.declare_parameter(
            "latest_file", "/tmp/computer_pi_camera/latest.jpg"
        ).value
        reconnect_period = node.declare_parameter("reconnect_period", 2.0).value
        jpeg_quality = node.declare_parameter("jpeg_quality", 85).value
        if not 1 <= port <= 65535 or reconnect_period <= 0 or not 1 <= jpeg_quality <= 100:
            raise ValueError(
                "port must be 1–65535, reconnect_period must be positive, "
                "and jpeg_quality must be 1–100"
            )
        publisher = node.create_publisher(
            CompressedImage, topic, qos_profile_sensor_data
        )
        source_address = (local_ip, 0) if local_ip else None
        node.get_logger().info(
            f"Receiving camera frames from {host}:{port} on {local_ip or 'automatic'}"
        )
        while rclpy.ok():
            try:
                with socket.create_connection(
                    (host, port), timeout=3.0, source_address=source_address
                ) as connection:
                    connection.settimeout(1.0)
                    node.get_logger().info("Camera TCP connected")
                    received_frames = 0
                    while rclpy.ok():
                        header, payload = read_frame(connection, rclpy.ok)
                        jpeg = payload_to_jpeg(header, payload, jpeg_quality)
                        received_frames += 1
                        if received_frames == 1:
                            node.get_logger().info(
                                "First camera frame: protocol=v%s %dx%d %s, "
                                "raw=%d bytes, jpeg=%d bytes"
                                % (
                                    header["version"],
                                    header["width"],
                                    header["height"],
                                    header["encoding"],
                                    len(payload),
                                    len(jpeg),
                                )
                            )
                        message = CompressedImage()
                        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                            header["stamp_ns"], 1_000_000_000
                        )
                        message.header.frame_id = header["frame_id"]
                        source_encoding = header.get("source_encoding", header["encoding"])
                        message.format = f"jpeg; source={source_encoding}"
                        message.data = jpeg
                        publisher.publish(message)
                        if latest_file:
                            atomic_write(latest_file, jpeg)
                            metadata_file = os.path.splitext(latest_file)[0] + ".json"
                            atomic_write(
                                metadata_file,
                                json.dumps(header, ensure_ascii=False).encode("utf-8"),
                            )
            except (OSError, EOFError, CameraProtocolError) as exc:
                if rclpy.ok():
                    node.get_logger().warn(f"Camera TCP disconnected: {exc}")
            except InterruptedError:
                break
            if rclpy.ok():
                time.sleep(reconnect_period)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
