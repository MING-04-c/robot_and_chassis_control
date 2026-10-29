#!/usr/bin/env python3
"""保留旧任务入口；海康通信的唯一实现位于 eli_cs_robot_serial/device/industrial_pc。"""

from industrial_pc_hk_transfer import (  # noqa: F401
    MAGIC, JPEG_PROTOCOL_VERSION, RAW_PROTOCOL_VERSION,
    DEFAULT_BIND_IP, DEFAULT_ALLOWED_CLIENT_IP,
    LatestFrame, PhotoQueue, ImageTcpServer, ImageClientHandler,
    CameraImageTcpNode, main,
)


if __name__ == "__main__":
    main()
