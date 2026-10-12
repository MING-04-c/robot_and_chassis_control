#!/usr/bin/env python3

from collections import deque
import csv
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node


class SensorPlot(Node):
    def __init__(self):
        super().__init__("sensor_plot")
        default_data_directory = Path(__file__).resolve().parents[1] / "data"
        self.data_directory = Path(self.declare_parameter(
            "data_directory", str(default_data_directory)).value)
        self.data_directory.mkdir(parents=True, exist_ok=True)
        filename = datetime.now().strftime("sensor_%Y%m%d_%H%M%S.csv")
        self.data_file = (self.data_directory / filename).open(
            "w", newline="", encoding="utf-8")
        self.data_writer = csv.writer(self.data_file)
        self.data_writer.writerow(["time", "Fx", "Fy", "Fz", "Mx", "My", "Mz"])
        self.data_file.flush()
        self.window_size = 500
        self.samples = [deque(maxlen=self.window_size) for _ in range(6)]
        self.subscription = self.create_subscription(
            WrenchStamped, "/sensor/wrench", self.on_wrench, 50)

        self.figure, self.axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        self.figure.canvas.manager.set_window_title("Six-axis Force/Torque Sensor")
        self.lines = []
        labels = ["Fx (N)", "Fy (N)", "Fz (N)", "Mx (N m)", "My (N m)", "Mz (N m)"]
        for axis, label in zip(self.axes.flat, labels):
            line, = axis.plot([], [], linewidth=1.2)
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
            self.lines.append(line)
        self.axes[-1, 0].set_xlabel("Sample")
        self.axes[-1, 1].set_xlabel("Sample")
        self.figure.tight_layout()

    def on_wrench(self, message):
        values = [
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z,
        ]
        for series, value in zip(self.samples, values):
            series.append(value)
        self.data_writer.writerow([
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            *values,
        ])
        self.data_file.flush()

    def update_plot(self):
        for axis, line, series in zip(self.axes.flat, self.lines, self.samples):
            values = list(series)
            line.set_data(range(len(values)), values)
            axis.relim()
            axis.autoscale_view()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()


def main():
    rclpy.init()
    node = SensorPlot()
    plt.ion()
    try:
        while rclpy.ok() and plt.fignum_exists(node.figure.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_plot()
            plt.pause(0.01)
    finally:
        node.data_file.close()
        node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
