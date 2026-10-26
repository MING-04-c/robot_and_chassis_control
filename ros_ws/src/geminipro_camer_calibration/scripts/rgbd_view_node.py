#!/usr/bin/env python3
"""Gemini Pro 实时彩色/深度预览、同 SDK FrameSet 拍照和 RGB 棋盘格标定，不控制机器人。"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from orbbec_camera_msgs.srv import GetCaptureFrame
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rgb_calibration import calibrate_rgb
from capture_v54 import RetryCapture, save_capture


def stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def info_dict(message):
    """将这一帧的 ROS CameraInfo 原样转为可保存的参数快照。"""
    return {
        "stamp_ns": stamp(message),
        "frame_id": message.header.frame_id,
        "width": message.width,
        "height": message.height,
        "distortion_model": message.distortion_model,
        "k": list(message.k),
        "d": list(message.d),
        "r": list(message.r),
        "p": list(message.p),
    }


class RGBDView(Node):
    def __init__(self):
        super().__init__("geminipro_calibration")
        cv2.setNumThreads(1)
        namespace = str(
            self.declare_parameter("camera_namespace", "camera").value
        ).strip("/")
        self.show_window = bool(self.declare_parameter("show_window", True).value)
        self.output = Path(
            str(self.declare_parameter("output_dir", "~/geminipro_captures").value)
        ).expanduser()
        self.slop = float(self.declare_parameter("pair_slop_sec", 0.05).value)
        self.timeout = float(self.declare_parameter("image_timeout_sec", 1.0).value)
        self.warmup = int(self.declare_parameter("warmup_pairs", 5).value)
        if not 0 < self.slop <= 0.2 or self.timeout <= 0 or self.warmup < 0:
            raise ValueError("配对时间差、超时或预热帧数无效")
        self.bridge = CvBridge()
        self.latest = {"color": None, "depth": None}
        self.times = {"color": deque(maxlen=120), "depth": deque(maxlen=120)}
        self.infos = {"color": deque(maxlen=30), "depth": deque(maxlen=30)}
        self.pairs = deque(maxlen=10)
        self.pair_count = 0
        self.notice = ""
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.rgb_image = self.depth_image = None
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            f"/{namespace}/color/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            f"/{namespace}/depth/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.color_sub.registerCallback(lambda message: self.on_frame("color", message))
        self.depth_sub.registerCallback(lambda message: self.on_frame("depth", message))
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=self.slop
        )
        self.sync.registerCallback(self.on_pair)
        for kind in ("color", "depth"):
            self.create_subscription(
                CameraInfo,
                f"/{namespace}/{kind}/camera_info",
                lambda message, kind=kind: self.infos[kind].append(message),
                qos_profile_sensor_data,
            )
        self.capture_group = ReentrantCallbackGroup()
        self.capture_client = self.create_client(
            GetCaptureFrame,
            f"/{namespace}/capture_frame",
            callback_group=self.capture_group,
        )
        self.capture_future = None
        self.sdk_future = None
        self.capture_busy = False
        self.create_service(
            Trigger, "~/capture", self.on_capture, callback_group=self.capture_group
        )
        self.root = None
        if self.show_window:
            self.create_panel()
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"Gemini Pro 预览已启动，订阅 /{namespace}/color 和 depth；不启动机器人"
        )

    def on_frame(self, kind, message):
        self.latest[kind] = message
        self.times[kind].append(time.monotonic())

    def on_pair(self, color, depth):
        # 消息时间为 ROS 秒/纳秒；不套用旧 SDK 未认证单位的时间戳阈值 100。
        if abs(stamp(color) - stamp(depth)) <= self.slop * 1e9:
            self.pairs.append((color, depth))
            self.pair_count += 1

    def fresh(self, message):
        if message is None:
            return False
        age = (self.get_clock().now().nanoseconds - stamp(message)) / 1e9
        return -0.1 <= age <= self.timeout

    def fps(self, kind):
        times = self.times[kind]
        now = time.monotonic()
        while times and times[0] < now - 1:
            times.popleft()
        if len(times) < 2 or now - times[-1] > self.timeout:
            return 0.0
        elapsed = times[-1] - times[0]
        return (len(times) - 1) / elapsed if elapsed > 0 else 0.0

    def paired_data(self):
        """仅用于预览状态：返回新鲜的 ROS 图像和 CameraInfo；不参与拍照保存。"""
        if self.pair_count < self.warmup or not self.pairs:
            raise ValueError(f"等待 RGB-D 配对与预热：{self.pair_count}/{self.warmup}")
        # DDS 不保证跨话题到达顺序；最新图像的 CameraInfo 可能仍在途中。
        # 只从短缓存中选择新鲜且参数完整的一对，不错配其他帧的参数。
        error = None
        for color, depth in reversed(self.pairs):
            try:
                return self.validate_pair(color, depth)
            except ValueError as reason:
                if error is None:
                    error = reason
        raise error

    def validate_pair(self, color, depth):
        """验证给定图像对，返回图像及各自同时间戳参数；不满足时抛出 ValueError。"""
        if not self.fresh(color) or not self.fresh(depth):
            raise ValueError("RGB-D 图像已超时")
        if (color.width, color.height, depth.width, depth.height) != (
            640,
            480,
            640,
            480,
        ):
            raise ValueError(
                f"实际 RGB/对齐深度为 {color.width}×{color.height}/{depth.width}×{depth.height}，与请求配置不符"
            )
        if color.header.frame_id != depth.header.frame_id:
            raise ValueError("深度未处于彩色光学坐标系，请检查软件 D2C 配置")
        if depth.encoding.lower() != "16uc1":
            raise ValueError("预览需要驱动发布的 16UC1 毫米深度")
        parameters = {}
        for kind, image in (("color", color), ("depth", depth)):
            info = next(
                (
                    item
                    for item in reversed(self.infos[kind])
                    if stamp(item) == stamp(image)
                    and item.header.frame_id == image.header.frame_id
                ),
                None,
            )
            if (
                info is None
                or (info.width, info.height) != (image.width, image.height)
                or info.k[0] <= 0
                or info.k[4] <= 0
                or not np.all(np.isfinite([*info.k, *info.d, *info.r, *info.p]))
            ):
                raise ValueError(f"等待 {kind} 同时间戳、有效的 CameraInfo")
            parameters[kind] = info_dict(info)
        return color, depth, parameters

    async def capture(self):
        """异步请求驱动下一组完整 SDK FrameSet，按主机格式保存，最多重试三次坏帧。"""
        if self.capture_busy:
            raise ValueError("拍照正在进行，请等待完成")
        if not self.capture_client.service_is_ready():
            raise ValueError(
                "SDK 拍照服务未就绪，请启动更新后的驱动并启用 enable_v54_capture"
            )
        self.capture_busy = True
        session = self.output / f"camera_v5_4_{time.time_ns()}"
        try:
            for attempt in range(1, 4):
                self.sdk_started = time.monotonic()
                self.sdk_future = self.capture_client.call_async(
                    GetCaptureFrame.Request()
                )
                response = await self.sdk_future
                if response is None:
                    raise ValueError("SDK 拍照服务超时或已取消")
                try:
                    directory = save_capture(
                        response, session / f"attempt_{attempt:02d}"
                    )
                except RetryCapture as error:
                    self.notice = f"第 {attempt} 次采集需重试：{error}"
                    if attempt == 3:
                        raise ValueError(
                            "连续三次采集未获得有效 RGB-D：" + str(error)
                        ) from error
                    continue
                self.notice = "已保存主机兼容 RGB-D：" + str(directory)
                return self.notice
        finally:
            self.capture_busy = False
            self.sdk_future = None

    async def on_capture(self, request, response):
        try:
            response.message = await self.capture()
            response.success = True
        except Exception as error:
            response.message = str(error)
            response.success = False
        return response

    def create_panel(self):
        import tkinter as tk
        from tkinter import ttk

        self.root = tk.Tk()
        self.root.title("Gemini Pro：RGB-D 画面、拍照与 RGB 内参标定")
        self.closed = False
        self.root.protocol("WM_DELETE_WINDOW", self.close_panel)
        self.summary = tk.StringVar(value="等待相机 RGB 和深度图像")
        ttk.Label(self.root, textvariable=self.summary).pack(padx=8, pady=6)
        self.parameters_text = tk.StringVar(value="等待当前相机内参")
        ttk.Label(self.root, textvariable=self.parameters_text).pack(padx=8, pady=3)
        images = ttk.Frame(self.root)
        images.pack()
        self.labels = [ttk.Label(images), ttk.Label(images)]
        for col, label in enumerate(self.labels):
            label.grid(row=0, column=col)
        controls = ttk.Frame(self.root)
        controls.pack(fill="x", padx=8, pady=6)
        ttk.Button(
            controls, text="保存一组 RGB-D 与参数", command=self.gui_capture
        ).grid(row=0, column=0)
        self.board = []
        for index, title in enumerate(("内角点列", "内角点行", "格长 mm")):
            ttk.Label(controls, text=title).grid(row=0, column=index * 2 + 1)
            value = tk.StringVar(value="0")
            self.board.append(value)
            ttk.Entry(controls, textvariable=value, width=7).grid(
                row=0, column=index * 2 + 2
            )
        ttk.Button(controls, text="计算 RGB 内参", command=self.gui_calibrate).grid(
            row=1, column=0, pady=5
        )
        self.feedback = tk.StringVar(
            value="左：RGB；右：对齐深度伪彩。相机出厂内参来自当前 CameraInfo；不执行手眼标定。"
        )
        ttk.Label(self.root, textvariable=self.feedback, wraplength=1200).pack(
            padx=8, pady=5
        )

    def gui_capture(self):
        if self.capture_busy or self.capture_future is not None:
            self.feedback.set("拍照正在进行，请等待完成")
            return
        self.capture_future = self.executor.create_task(self.capture())
        self.feedback.set("正在请求同一个 SDK FrameSet，并读取当次参数与原始深度比例")

    def gui_calibrate(self):
        try:
            if self.future:
                raise ValueError("RGB 标定正在计算")
            cols, rows, size = (
                int(self.board[0].get()),
                int(self.board[1].get()),
                float(self.board[2].get()),
            )
            if cols < 2 or rows < 2 or not np.isfinite(size) or size <= 0:
                raise ValueError("请填写真实内角点列、行和实测格长毫米")
            output = self.output / f"calibration_{time.time_ns()}"
            self.future = self.worker.submit(
                calibrate_rgb, self.output, output, cols, rows, size
            )
            self.feedback.set("正在后台计算 RGB 内参；不会替换设备出厂的深度/配准参数")
        except ValueError as error:
            self.feedback.set(str(error))

    def close_panel(self):
        self.closed = True
        self.root.destroy()

    def tick(self):
        if self.sdk_future and not self.sdk_future.done():
            if time.monotonic() - self.sdk_started > 25:
                self.sdk_future.cancel()
        if self.capture_future and self.capture_future.done():
            try:
                self.notice = self.capture_future.result()
            except Exception as error:
                self.notice = "拍照失败：" + str(error)
            self.capture_future = None
        if self.future and self.future.done():
            try:
                path, rms = self.future.result()
                self.notice = f"RGB 内参计算完成，RMS={rms:.3f} px；结果：{path}"
            except Exception as error:
                self.notice = "RGB 标定失败：" + str(error)
            self.future = None
        color, depth = self.latest["color"], self.latest["depth"]
        try:
            ready_message = "RGB-D 预览就绪"
            _, _, parameters = self.paired_data()
            ready = self.capture_client.service_is_ready()
            if not ready:
                ready_message = "预览就绪，等待 SDK 拍照服务"
        except ValueError as error:
            ready, ready_message = False, str(error)
        status = {
            "ready": ready,
            "message": ready_message,
            "color_fps": self.fps("color"),
            "depth_fps": self.fps("depth"),
            "pair_count": self.pair_count,
            "capture_or_calibration": self.notice,
            "capture_busy": self.capture_busy,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if not self.root:
            return
        from PIL import Image as PILImage, ImageTk

        self.summary.set(
            f'RGB {status["color_fps"]:.1f} FPS | 深度 {status["depth_fps"]:.1f} FPS | '
            + ready_message
        )
        if ready:
            k = parameters["color"]["k"]
            self.parameters_text.set(
                f"当前 RGB 内参：fx={k[0]:.3f}, fy={k[4]:.3f}, "
                f"cx={k[2]:.3f}, cy={k[5]:.3f} px；来自本次 CameraInfo"
            )
        else:
            self.parameters_text.set("等待当前帧与有效内参匹配")
        previews = []
        try:
            rgb = (
                self.bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
                if self.fresh(color)
                else np.zeros((480, 640, 3), np.uint8)
            )
            previews.append(rgb)
            if self.fresh(depth):
                raw = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
                millimeters = (
                    raw * 1000.0
                    if depth.encoding == "32FC1"
                    else raw.astype(np.float32)
                )
                valid = np.isfinite(millimeters) & (millimeters > 0)
                normalized = np.clip(
                    np.nan_to_num(millimeters) / 3000 * 255, 0, 255
                ).astype(np.uint8)
                visual = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
                visual[~valid] = 0
                h, w = raw.shape
                z = millimeters[h // 2, w // 2]
                cv2.drawMarker(visual, (w // 2, h // 2), (255, 255, 255), markerSize=15)
                cv2.putText(
                    visual,
                    (
                        f"Center Z: {z:.0f} mm"
                        if valid[h // 2, w // 2]
                        else "Center: invalid depth"
                    ),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
                previews.append(visual)
            else:
                previews.append(np.zeros((480, 640, 3), np.uint8))
            self.photos = []
            for label, image in zip(self.labels, previews):
                image = cv2.resize(image, (600, 450))
                photo = ImageTk.PhotoImage(
                    PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                )
                self.photos.append(photo)
                label.configure(image=photo)
        except (CvBridgeError, cv2.error, ValueError) as error:
            self.notice = str(error)
        if self.notice:
            self.feedback.set(self.notice)
        self.root.update_idletasks()
        self.root.update()
        if self.closed:
            raise KeyboardInterrupt

    def destroy_node(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
        if self.root and not self.closed:
            self.close_panel()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RGBDView()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
