#!/usr/bin/env python3
"""显示传感器六维曲线；原始观察数据与 SI 控制数据明确区分。"""

from collections import deque
import csv
from datetime import datetime
import math
from pathlib import Path
import signal
from time import monotonic

import matplotlib
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray, String


class SensorPlot(Node):
    def __init__(self):
        super().__init__("sensor_plot")
        self.data_source = self.declare_parameter("data_source", "auto").value
        if self.data_source not in ("auto", "verified", "raw"):
            raise ValueError("data_source 必须为 auto、verified 或 raw")
        default_directory = Path(__file__).resolve().parents[1] / "data"
        self.data_directory = Path(self.declare_parameter(
            "data_directory", str(default_directory)).value).expanduser()
        self.data_directory.mkdir(parents=True, exist_ok=True)
        filename = datetime.now().strftime("sensor_%Y%m%d_%H%M%S.csv")
        self.data_file = (self.data_directory / filename).open(
            "w", newline="", encoding="utf-8")
        self.data_writer = csv.writer(self.data_file)
        self.data_writer.writerow([
            "time", "Fx", "Fy", "Fz", "Mx", "My", "Mz",
            "source", "device_unit", "unit_state",
        ])
        self.data_file.flush()
        self.samples = [deque(maxlen=500) for _ in range(6)]
        self.have_data = False
        self.dirty = True
        self.source = None
        self.series_unit = None
        self.unit_state = "UNKNOWN"
        self.device_unit = "unknown"
        self.last_sample_at = -math.inf
        self.subscription = self.create_subscription(
            WrenchStamped, "/sensor/wrench", self.on_wrench, qos_profile_sensor_data)
        self.raw_subscription = self.create_subscription(
            Float64MultiArray, "/sensor/raw_values", self.on_raw, qos_profile_sensor_data)
        self.unit_subscription = self.create_subscription(
            String, "/sensor/unit_status", self.on_unit_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.figure, self.axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        if not getattr(self.figure.canvas, "required_interactive_framework", None):
            self.data_file.close()
            plt.close(self.figure)
            raise RuntimeError(
                f"绘图后端 {matplotlib.get_backend()} 不支持窗口；请检查图形桌面、DISPLAY 和 MPLBACKEND")
        self.figure.canvas.manager.set_window_title("Six-axis Force/Torque Sensor")
        self.lines = []
        for axis, name in zip(self.axes.flat, ("Fx", "Fy", "Fz", "Mx", "My", "Mz")):
            line, = axis.plot([], [], linewidth=1.2)
            axis.set_ylabel(f"{name} (waiting)")
            axis.grid(True, alpha=0.3)
            self.lines.append(line)
        self.axes[-1, 0].set_xlabel("Sample")
        self.axes[-1, 1].set_xlabel("Sample")
        self.title = self.figure.suptitle("Waiting for sensor data")
        self.footer = self.figure.text(
            0.5, 0.01, "Raw observation data is not control input", ha="center")
        self.figure.tight_layout(rect=[0, 0.04, 1, 0.95])
        self.get_logger().info(
            f"绘图窗口已创建，backend={matplotlib.get_backend()}，data_source={self.data_source}；"
            f"订阅 /sensor/wrench 和 /sensor/raw_values；CSV={self.data_file.name}")

    def show_window(self):
        """显示窗口；没有有效 SI 数据时仍保留窗口和单位状态。"""
        plt.show(block=False)
        manager = self.figure.canvas.manager
        manager.show()
        window = getattr(manager, "window", None)
        if hasattr(window, "deiconify"):
            window.deiconify()
            window.lift()
        elif hasattr(window, "showNormal"):
            window.showNormal()
            window.raise_()
            window.activateWindow()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        self.get_logger().info("窗口已显示；auto 模式下单位未确认时显示原始读数，确认后显示 N/N·m")

    def on_unit_status(self, message):
        fields = {}
        for part in message.data.split(";"):
            key, separator, value = part.strip().partition("=")
            if separator:
                fields[key] = value
        self.unit_state = fields.get("state", "UNKNOWN")
        self.device_unit = fields.get("device_unit", "unknown")
        self.dirty = True
        self.get_logger().info(f"传感器单位状态：{message.data}")

    def append_sample(self, values, source, timestamp):
        """写入曲线与 CSV；切换来源/单位时清空曲线，避免混画不同量纲。"""
        if len(values) != 6 or not all(math.isfinite(x) for x in values):
            return
        unit = "SI" if source == "verified" else self.device_unit
        if source != self.source or unit != self.series_unit:
            for series in self.samples:
                series.clear()
            self.source, self.series_unit = source, unit
            self.get_logger().info(
                "显示已确认的 N/N·m 数据" if source == "verified" else
                f"显示原始观察数据，设备单位={self.device_unit}；不用于机器人控制")
        for series, value in zip(self.samples, values):
            series.append(value)
        self.have_data = True
        self.last_sample_at = monotonic()
        self.dirty = True
        self.data_writer.writerow([
            timestamp, *values, source, self.device_unit, self.unit_state,
        ])
        self.data_file.flush()

    def on_raw(self, message):
        if self.data_source == "verified":
            return
        if self.data_source == "auto" and self.source == "verified":
            # 绘图较慢不能触发 RAW/SI 来回切换；单位状态明确失效时才退回原始数据。
            if self.unit_state in ("READY", "MANUAL", "UNKNOWN"):
                return
        self.append_sample(
            list(message.data), "raw", self.get_clock().now().nanoseconds / 1e9)

    def on_wrench(self, message):
        if self.data_source == "raw":
            return
        if self.unit_state not in ("READY", "MANUAL", "UNKNOWN"):
            return
        values = [
            message.wrench.force.x, message.wrench.force.y, message.wrench.force.z,
            message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z,
        ]
        self.append_sample(values, "verified",
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    def update_plot(self):
        stale = self.have_data and monotonic() - self.last_sample_at > 1.0
        mode = "SI: N / N m" if self.source == "verified" else "RAW: device units / UNVERIFIED"
        prefix = "No new data" if stale else ("Live" if self.have_data else "Waiting for sensor data")
        title = f"{prefix} | {mode} | unit state={self.unit_state}"
        if not self.dirty and self.title.get_text() == title:
            return
        self.dirty = False
        self.title.set_text(title)
        self.footer.set_text(
            "Source: /sensor/wrench (SI)" if self.source == "verified" else
            "Source: /sensor/raw_values | observation only, not control input")
        names = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")
        for i, (axis, line, series) in enumerate(zip(self.axes.flat, self.lines, self.samples)):
            unit = ("N" if i < 3 else "N m") if self.source == "verified" else "raw"
            axis.set_ylabel(f"{names[i]} ({unit})")
            values = list(series)
            line.set_data(range(len(values)), values)
            axis.relim()
            axis.autoscale_view()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()


def main():
    # 不在 Tk 的绘图回调中抛 KeyboardInterrupt；让主循环收到退出标记后统一清理。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    previous = {}

    def request_stop(_signum, _frame):
        stopping[0] = True

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, request_stop)
    plt.ion()
    node = None
    try:
        node = SensorPlot()
        node.show_window()
        last_draw = 0.0
        while not stopping[0] and rclpy.ok() and plt.fignum_exists(node.figure.number):
            rclpy.spin_once(node, timeout_sec=0.02)
            if monotonic() - last_draw >= 0.1:
                node.update_plot()
                last_draw = monotonic()
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.data_file.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        plt.close("all")
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main()
