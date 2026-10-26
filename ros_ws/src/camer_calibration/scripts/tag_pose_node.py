#!/usr/bin/env python3
"""订阅海康图像，提供 AprilTag 相对位姿、可视化及棋盘格标定采集。"""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from calibrate_camera import detect_corners, run_calibration, save_image
from tag_geometry import (
    FAMILIES,
    estimate_tag,
    load_intrinsics,
    make_detector,
    quaternion_from_rvec,
)


class TagPoseNode(Node):
    def __init__(self):
        super().__init__("camera_calibration")
        cv2.setNumThreads(1)
        defaults = {
            "image_topic": "/camera/image",
            "camera_frame": "camera_optical_frame",
            "calibration_file": "",
            "tag_family": "auto",
            "tag_id": -1,
            "tag_size_mm": 0.0,
            "show_window": True,
            "process_rate_hz": 10.0,
            "image_timeout_sec": 0.5,
            "max_reprojection_error_px": 3.0,
            "board_cols": 0,
            "board_rows": 0,
            "square_size_mm": 0.0,
            "output_dir": "~/camera_calibration",
            "min_views": 10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.mutable = (
            "calibration_file",
            "tag_family",
            "tag_id",
            "tag_size_mm",
            "board_cols",
            "board_rows",
            "square_size_mm",
        )
        self.settings = {name: self.get_parameter(name).value for name in self.mutable}
        self.intrinsics = None
        self.detectors = {}
        self.configure(self.settings)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.frame = str(self.get_parameter("camera_frame").value)
        self.timeout = float(self.get_parameter("image_timeout_sec").value)
        self.max_error = float(self.get_parameter("max_reprojection_error_px").value)
        rate = float(self.get_parameter("process_rate_hz").value)
        self.min_views = int(self.get_parameter("min_views").value)
        if (
            not self.frame
            or any(
                not math.isfinite(v) or v <= 0
                for v in (rate, self.timeout, self.max_error)
            )
            or self.min_views < 3
        ):
            raise ValueError("相机坐标系、处理频率、超时、误差阈值或最少视图数量无效")
        self.output = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.bridge = CvBridge()
        self.latest = None
        self.latest_at = 0.0
        self.generation = 0
        self.receive_times = deque(maxlen=4096)
        self.process_times = deque(maxlen=4096)
        self.processed = -1
        self.raw_image = None
        self.annotated = None
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.calibration_note = ""
        self.last_calibration_message = ""
        self.status = self.invalid("waiting_image", "等待海康相机图像")
        self.image_pub = self.create_publisher(
            Image, "~/debug_image", qos_profile_sensor_data
        )
        self.pose_pub = self.create_publisher(PoseStamped, "~/tag_pose", 10)
        self.status_pub = self.create_publisher(String, "~/tag_status", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "~/markers", 10)
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )
        self.create_service(Trigger, "~/save_frame", self.save_frame_service)
        self.create_service(Trigger, "~/capture", self.capture_service)
        self.create_service(Trigger, "~/calibrate", self.calibrate_service)
        self.create_service(Trigger, "~/reload_calibration", self.reload_service)
        self.panel = None
        if self.get_parameter("show_window").value:
            from tag_panel import TagPanel

            self.panel = TagPanel(self)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            "实时 AprilTag 界面已启动；没有真实内参或实测 Tag 边长时只检测，不输出距离"
        )

    def configure(self, settings):
        """校验可编辑参数，并一次性更新检测器和内参；零尺寸表示尚未设置。"""
        if settings["tag_family"] not in ("auto", *FAMILIES):
            raise ValueError("未知的 AprilTag 家族")
        for name in ("tag_size_mm", "square_size_mm"):
            value = settings[name]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError(name + " 必须是非负有限数值")
        for name in ("board_cols", "board_rows", "tag_id"):
            value = settings[name]
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value < (-1 if name == "tag_id" else 0)
            ):
                raise ValueError(name + " 的整数值无效")
        intrinsics = (
            load_intrinsics(settings["calibration_file"])
            if settings["calibration_file"]
            else None
        )
        families = (
            list(FAMILIES)
            if settings["tag_family"] == "auto"
            else [settings["tag_family"]]
        )
        detectors = {family: make_detector(family) for family in families}
        self.settings = settings.copy()
        self.intrinsics = intrinsics
        self.detectors = detectors
        if hasattr(self, "status"):
            self.status = self.invalid("settings_updated", "设置已更新，等待新的测量")
            self.processed = -1

    def on_parameters(self, parameters):
        settings = self.settings.copy()
        for parameter in parameters:
            if parameter.name not in self.mutable:
                return SetParametersResult(
                    successful=False, reason="该参数需要重新 launch：" + parameter.name
                )
            settings[parameter.name] = parameter.value
        try:
            self.configure(settings)
        except (ValueError, OSError, KeyError, TypeError, cv2.error) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def on_image(self, message):
        # 回调仅缓存最新消息；低频定时处理，避免高帧率输入排队。
        now = time.monotonic()
        if now - self.latest_at > self.timeout:
            self.receive_times.clear()
            self.process_times.clear()
        self.latest = message
        self.latest_at = now
        self.receive_times.append(now)
        self.generation += 1

    def rolling_fps(self, timestamps, now):
        """最近一秒的接收/处理事件间隔估计 FPS；样本不足或超时返回零。"""
        while timestamps and timestamps[0] < now - 1.0:
            timestamps.popleft()
        if len(timestamps) < 2 or now - timestamps[-1] > self.timeout:
            return 0.0
        elapsed = timestamps[-1] - timestamps[0]
        return (len(timestamps) - 1) / elapsed if elapsed > 0 else 0.0

    def invalid(self, state, message):
        """无效状态显式清空坐标，防止 GUI 继续显示上次测量值。"""
        return {
            "valid": False,
            "state": state,
            "message": message,
            "xyz_mm": None,
            "distance_mm": None,
        }

    def fresh(self):
        if self.latest is None or time.monotonic() - self.latest_at > self.timeout:
            return False
        stamp = self.latest.header.stamp
        age = (
            self.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec
        ) / 1e9
        return -0.1 <= age <= self.timeout

    def process(self):
        message = self.latest
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8").copy()
        self.raw_image = image.copy()
        self.annotated = image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = []
        for family, detector in self.detectors.items():
            corners, ids, _ = detector(gray)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                found.extend(
                    (family, int(tag_id), corner)
                    for tag_id, corner in zip(ids.ravel(), corners)
                )
        selected = [
            item
            for item in found
            if self.settings["tag_id"] < 0 or item[1] == self.settings["tag_id"]
        ]
        label = ", ".join(f"{family} ID={tag_id}" for family, tag_id, _ in found)
        if not selected:
            if found:
                return self.invalid(
                    "target_id_mismatch",
                    f"已识别 {label}，但设置的目标 ID={self.settings['tag_id']} 不匹配；可设 ID=-1 自动选择单个目标",
                )
            return self.invalid(
                "not_detected",
                "未识别到 AprilTag；确保四角和白边完整入镜，可点击“保存当前原图”进一步检查",
            )
        if len(selected) != 1:
            return self.invalid(
                "ambiguous_target", "识别到多个候选，请指定家族和 ID：" + label
            )
        family, tag_id, corners = selected[0]
        cv2.putText(
            image,
            f"{family} ID={tag_id}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 220, 0),
            2,
        )
        pixels = np.asarray(corners).reshape(4, 2)
        height, width = gray.shape
        if (
            np.any(pixels[:, 0] < 6)
            or np.any(pixels[:, 0] >= width - 6)
            or np.any(pixels[:, 1] < 6)
            or np.any(pixels[:, 1] >= height - 6)
        ):
            return self.invalid(
                "target_at_border",
                f"已识别 {family} ID={tag_id}，但黑框过于靠边或已被裁切；移动标记，保留完整白边后再测距",
            )
        if self.intrinsics is None or self.settings["tag_size_mm"] <= 0:
            return self.invalid(
                "need_calibration_or_size",
                f"已识别 {family} ID={tag_id}；请选择真实内参并填写实测黑框边长",
            )
        if message.header.frame_id != self.frame:
            return self.invalid(
                "frame_mismatch", "图像坐标系与 camera_frame 不一致，请重新配置 launch"
            )
        matrix, distortion, size = self.intrinsics
        if (image.shape[1], image.shape[0]) != size:
            return self.invalid(
                "resolution_mismatch", f"图像分辨率不匹配内参 {size}，不自动缩放内参"
            )
        rotation, translation, rvec, error = estimate_tag(
            corners, self.settings["tag_size_mm"], matrix, distortion
        )
        if error > self.max_error:
            return self.invalid(
                "high_reprojection_error",
                f"重投影 RMS {error:.2f} px 偏高，暂不发布位姿",
            )
        cv2.drawFrameAxes(
            image,
            matrix,
            distortion,
            rvec,
            translation,
            self.settings["tag_size_mm"] * 0.5,
        )
        distance = float(np.linalg.norm(translation))
        cv2.putText(
            image,
            f"XYZ mm: {translation[0]:.1f}, {translation[1]:.1f}, {translation[2]:.1f}  D: {distance:.1f}",
            (15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 220, 0),
            2,
        )
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
            float, translation / 1000
        )
        q = quaternion_from_rvec(rvec)
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = q
        self.pose_pub.publish(pose)
        self.publish_markers(pose, rotation, distance)
        return {
            "valid": True,
            "state": "tracking",
            "message": f"{family} ID={tag_id}，RMS={error:.3f} px；坐标为 Tag 中心在相机光学坐标系中的位置",
            "tag_family": family,
            "tag_id": tag_id,
            "xyz_mm": translation.tolist(),
            "distance_mm": distance,
            "reprojection_rms_px": error,
        }

    def publish_markers(self, pose, rotation, distance):
        """在相机光学坐标系绘制中心连线、Tag 三轴和数值；ROS 长度统一为米。"""
        position = np.array(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        )
        markers = []

        def marker(index, kind):
            item = Marker()
            item.header = pose.header
            item.ns = "camera_tag"
            item.id = index
            item.type = kind
            item.action = Marker.ADD
            item.pose.orientation.w = 1.0
            item.color.a = 1.0
            item.lifetime.sec, item.lifetime.nanosec = 0, int(
                min(self.timeout, 0.9) * 1e9
            )
            markers.append(item)
            return item

        def point(values):
            return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))

        line = marker(0, Marker.LINE_LIST)
        line.scale.x = 0.002
        line.color.g, line.color.b = 1.0, 1.0
        line.points = [point(np.zeros(3)), point(position)]
        for axis in range(3):
            item = marker(axis + 1, Marker.LINE_LIST)
            item.scale.x = 0.003
            setattr(item.color, ("r", "g", "b")[axis], 1.0)
            item.points = [
                point(position),
                point(
                    position + rotation[:, axis] * self.settings["tag_size_mm"] / 2000
                ),
            ]
        text = marker(4, Marker.TEXT_VIEW_FACING)
        text.pose.position = point(position + np.array([0, -0.03, 0]))
        text.scale.z = 0.02
        text.color.r = text.color.g = text.color.b = 1.0
        text.text = f"XYZ: {position[0]*1000:.1f}, {position[1]*1000:.1f}, {position[2]*1000:.1f} mm\nD={distance:.1f} mm"
        self.markers_pub.publish(MarkerArray(markers=markers))

    def tick(self):
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                path = str(Path(result["config"]["output_dir"]) / "calibration.yaml")
                outcome = self.set_parameters_atomically(
                    [Parameter("calibration_file", value=path)]
                )
                if not outcome.successful:
                    raise ValueError(outcome.reason)
                self.calibration_note = "内参已计算并加载：" + path
                if self.panel:
                    self.panel.values["calibration_file"].set(path)
            except Exception as error:
                self.calibration_note = "标定失败：" + str(error)
            self.last_calibration_message = self.calibration_note
            self.future = None
        if not self.fresh():
            self.status = self.invalid(
                "stale_image", "未收到新鲜图像，测量已清空；检查相机或图像时间戳"
            )
            if self.raw_image is not None:
                self.annotated = self.raw_image.copy()
                cv2.putText(
                    self.annotated,
                    "STALE IMAGE - NO MEASUREMENT",
                    (15, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
        elif self.processed != self.generation:
            try:
                self.status = self.process()
                self.process_times.append(time.monotonic())
            except (CvBridgeError, ValueError, cv2.error) as error:
                self.status = self.invalid("processing_error", str(error))
            self.processed = self.generation
        now = time.monotonic()
        self.status["receive_fps"] = self.rolling_fps(self.receive_times, now)
        self.status["process_fps"] = (
            self.rolling_fps(self.process_times, now) if self.fresh() else 0.0
        )
        self.status["received_frames"] = self.generation
        self.status["calibration_running"] = self.future is not None
        self.status["calibration_message"] = self.last_calibration_message
        self.status["frame_id"] = self.frame
        self.status["image_stamp"] = (
            {
                "sec": self.latest.header.stamp.sec,
                "nanosec": self.latest.header.stamp.nanosec,
            }
            if self.latest
            else None
        )
        if not self.status["valid"]:
            self.markers_pub.publish(
                MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
            )
        self.status_pub.publish(
            String(data=json.dumps(self.status, ensure_ascii=False, allow_nan=False))
        )
        display_image = self.annotated
        if display_image is not None and self.latest:
            display_image = display_image.copy()
            height, width = display_image.shape[:2]
            cv2.rectangle(
                display_image,
                (0, max(0, height - 35)),
                (width, height),
                (30, 30, 30),
                -1,
            )
            cv2.putText(
                display_image,
                f"RX: {self.status['receive_fps']:.1f} FPS | Process: {self.status['process_fps']:.1f} FPS | Frames: {self.generation}",
                (10, max(20, height - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
            debug = self.bridge.cv2_to_imgmsg(display_image, encoding="bgr8")
            debug.header = self.latest.header
            self.image_pub.publish(debug)
        if self.panel:
            self.panel.refresh(display_image, self.status)
            if self.calibration_note:
                self.panel.notice.set(self.calibration_note)
                self.calibration_note = ""
            if self.panel.closed:
                raise KeyboardInterrupt

    def save_frame(self):
        """保存新鲜相机原图与检测设置，独立于棋盘格采集，不包含界面缩放或叠字。"""
        if not self.fresh():
            raise ValueError("没有新鲜图像，不能保存原图")
        message = self.latest
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        path = self.output / "diagnostics" / f"camera_{time.time_ns()}.png"
        save_image(path, image)
        metadata = {
            "settings": self.settings,
            "status": self.status,
            "image_width": image.shape[1],
            "image_height": image.shape[0],
            "frame_id": message.header.frame_id,
            "stamp": {
                "sec": message.header.stamp.sec,
                "nanosec": message.header.stamp.nanosec,
            },
        }
        path.with_suffix(".json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        return "已保存原图与检测设置：" + str(path)

    def save_frame_service(self, request, response):
        return self.service_result(response, self.save_frame)

    def dataset(self):
        cols, rows, size = (
            self.settings[k] for k in ("board_cols", "board_rows", "square_size_mm")
        )
        if cols < 2 or rows < 2 or size <= 0:
            raise ValueError("请先填写真实棋盘格内角点列、行和实测格长；当前值未设置")
        return self.output / f"photos_{cols}x{rows}_{size:g}mm"

    def capture_sample(self):
        if self.future:
            raise ValueError("正在计算内参，请等待完成后再采集")
        directory = self.dataset()
        if not self.fresh() or self.latest is None:
            raise ValueError("没有新鲜图像，不能采集")
        image = self.bridge.imgmsg_to_cv2(self.latest, desired_encoding="bgr8")
        if (
            detect_corners(
                image, (self.settings["board_cols"], self.settings["board_rows"])
            )
            is None
        ):
            raise ValueError("未检测到完整棋盘格，照片未保存；AprilTag 不能用于此标定")
        path = directory / f"board_{time.time_ns()}.png"
        save_image(path, image)
        return f'已保存第 {len(list(directory.glob("*.png")))} 张：{path}'

    def start_calibration(self):
        if self.future:
            raise ValueError("内参正在计算，请勿重复启动")
        directory = self.dataset()
        if len(list(directory.glob("*.png"))) < self.min_views:
            raise ValueError(
                f"有效采集照片不足 {self.min_views} 张，请补拍不同位置、角度和距离"
            )
        config = {
            "image_dir": str(directory),
            "output_dir": str(self.output / f"results_{time.time_ns()}"),
            "validation_dir": None,
            "board_cols": self.settings["board_cols"],
            "board_rows": self.settings["board_rows"],
            "square_size_mm": self.settings["square_size_mm"],
            "min_views": self.min_views,
            "error_warning_px": None,
            "undistort_alpha": 1.0,
        }
        self.future = self.worker.submit(run_calibration, config)
        return "正在后台计算内参；完成后自动加载，结果保存在 " + config["output_dir"]

    def capture_service(self, request, response):
        return self.service_result(response, self.capture_sample)

    def calibrate_service(self, request, response):
        return self.service_result(response, self.start_calibration)

    def reload_service(self, request, response):
        def reload():
            self.configure(self.settings)
            return "已重新读取内参文件" if self.intrinsics else "尚未设置内参文件"

        return self.service_result(response, reload)

    @staticmethod
    def service_result(response, operation):
        try:
            response.message = operation()
            response.success = True
        except (ValueError, OSError, KeyError, cv2.error) as error:
            response.message = str(error)
            response.success = False
        return response

    def destroy_node(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
        if self.panel and not self.panel.closed:
            self.panel.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TagPoseNode()
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
