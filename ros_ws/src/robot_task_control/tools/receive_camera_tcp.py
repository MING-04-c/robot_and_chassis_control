#!/usr/bin/env python3
"""Receive MASC TCP image frames; save JPEG or exact raw ROS Image bytes."""

import argparse
import json
import os
from pathlib import Path
import socket
import struct
import time

MAGIC = b"MIMG"
MAX_HEADER_LENGTH = 65536
MAX_JPEG_LENGTH = 32 * 1024 * 1024
MAX_RAW_LENGTH = 128 * 1024 * 1024


def read_exact(connection, count):
    chunks = []
    remaining = count
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise EOFError("TCP connection closed during a frame")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def receive_frame(connection):
    prefix = read_exact(connection, 8)
    if prefix[:4] != MAGIC:
        raise ValueError("Invalid frame magic")
    header_length = struct.unpack("!I", prefix[4:])[0]
    if not 1 <= header_length <= MAX_HEADER_LENGTH:
        raise ValueError("Invalid header length")
    header = json.loads(read_exact(connection, header_length).decode("utf-8"))
    if header.get("type") != "image":
        raise ValueError("Unsupported message type")
    payload_length = header.get("payload_length")
    if header.get("version") == 1 and header.get("encoding") == "jpeg":
        if not isinstance(payload_length, int) or not 1 <= payload_length <= MAX_JPEG_LENGTH:
            raise ValueError("Invalid JPEG length")
    elif header.get("version") == 2:
        width, height, step = (header.get(key) for key in ("width", "height", "step"))
        if not all(isinstance(x, int) and x > 0 for x in (width, height, step)):
            raise ValueError("Invalid raw image dimensions")
        if header.get("is_bigendian") not in (0, 1):
            raise ValueError("Invalid byte order")
        if not isinstance(header.get("encoding"), str) or not header["encoding"]:
            raise ValueError("Missing ROS image encoding")
        if (not isinstance(payload_length, int) or payload_length != step * height
                or payload_length > MAX_RAW_LENGTH):
            raise ValueError("Invalid raw image length")
    else:
        raise ValueError("Unsupported image protocol version or encoding")
    return header, read_exact(connection, payload_length)


def write_latest(path, header, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_image = path.with_name(path.name + ".tmp")
    temporary_image.write_bytes(payload)
    os.replace(temporary_image, path)
    metadata_path = path.with_suffix(".json")
    temporary_metadata = metadata_path.with_name(metadata_path.name + ".tmp")
    temporary_metadata.write_text(json.dumps(header, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary_metadata, metadata_path)


def decode_for_display(header, payload, cv2, np):
    if header["version"] == 1:
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("JPEG decoding failed")
        return image
    encoding = header["encoding"].lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(encoding)
    if channels is None:
        raise ValueError("--show does not support %s; raw bytes were saved" % encoding)
    width, height, step = header["width"], header["height"], header["step"]
    if step < width * channels:
        raise ValueError("Raw row is shorter than image width")
    rows = np.frombuffer(payload, dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :width * channels].reshape(height, width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    if encoding == "mono8":
        return pixels[:, :, 0]
    return pixels


def main():
    parser = argparse.ArgumentParser(description="Receive MASC JPEG or raw images over direct TCP")
    parser.add_argument("--host", default="169.254.199.100", help="Camera computer Ethernet IP")
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--latest", type=Path, help="Overwrite this .jpg or .raw and its .json metadata")
    parser.add_argument("--save-dir", type=Path, help="Save every received frame")
    parser.add_argument("--show", action="store_true", help="Display supported image encodings with OpenCV")
    parser.add_argument("--count", type=int, default=0, help="Stop after N frames; 0 means continuous")
    args = parser.parse_args()
    if args.count < 0:
        parser.error("--count must be nonnegative")
    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)
    if args.show:
        import cv2
        import numpy as np

    received = 0
    try:
        while args.count == 0 or received < args.count:
            try:
                with socket.create_connection((args.host, args.port), timeout=5.0) as connection:
                    connection.settimeout(15.0)
                    print("Connected to %s:%d" % (args.host, args.port), flush=True)
                    while args.count == 0 or received < args.count:
                        header, payload = receive_frame(connection)
                        received += 1
                        suffix = ".jpg" if header["version"] == 1 else ".raw"
                        if args.latest:
                            write_latest(args.latest, header, payload)
                        if args.save_dir:
                            name = "frame_%08d_%d%s" % (
                                header["sequence"], header["stamp_ns"], suffix
                            )
                            write_latest(args.save_dir / name, header, payload)
                        print(
                            "frame %d: %dx%d, %s, %d bytes, timestamp %d"
                            % (
                                header["sequence"], header["width"], header["height"],
                                header["encoding"], len(payload), header["stamp_ns"]
                            ),
                            flush=True,
                        )
                        if args.show:
                            image = decode_for_display(header, payload, cv2, np)
                            cv2.imshow("Remote Hik camera", image)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                return
            except (OSError, EOFError) as exc:
                print("Disconnected: %s; reconnecting in 1 second" % exc, flush=True)
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
