#!/usr/bin/env python3
"""ROS 节点使用的中文可视化面板；只负责交互，不独立启动 ROS。"""

import tkinter as tk
from tkinter import filedialog, ttk

import cv2
from PIL import Image, ImageTk
from rclpy.parameter import Parameter

from tag_geometry import FAMILIES


class TagPanel:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("海康相机：内参标定与 AprilTag 距离")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.closed = False
        self.values = {}
        form = ttk.LabelFrame(
            self.root, text="AprilTag 测量设置（请使用实测边长与真实内参）"
        )
        form.pack(fill="x", padx=8, pady=4)
        for row, (name, label) in enumerate(
            (
                ("calibration_file", "标定 YAML"),
                ("tag_family", "Tag 家族"),
                ("tag_id", "Tag ID（-1：画面仅一个 Tag 时选择）"),
                ("tag_size_mm", "黑色外框边长 mm"),
            )
        ):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w")
            variable = tk.StringVar(value=str(node.get_parameter(name).value))
            self.values[name] = variable
            widget = (
                ttk.Combobox(
                    form,
                    textvariable=variable,
                    values=["auto", *FAMILIES],
                    state="readonly",
                )
                if name == "tag_family"
                else ttk.Entry(form, textvariable=variable, width=65)
            )
            widget.grid(row=row, column=1, sticky="ew")
        ttk.Button(form, text="选择内参文件", command=self.choose_file).grid(
            row=0, column=2
        )
        ttk.Button(form, text="应用测量设置", command=self.apply).grid(row=3, column=2)
        ttk.Button(form, text="保存当前原图", command=self.save_frame).grid(
            row=2, column=2
        )
        form.columnconfigure(1, weight=1)
        self.fps = tk.StringVar(
            value="相机接收：0.0 FPS    检测处理：0.0 FPS    累计接收：0 帧"
        )
        ttk.Label(self.root, textvariable=self.fps, font=("", 12)).pack(padx=8, pady=4)
        self.pose = tk.StringVar(value="尚无有效测量")
        ttk.Label(self.root, textvariable=self.pose, font=("", 15)).pack(padx=8, pady=4)
        self.image = ttk.Label(self.root)
        self.image.pack(padx=8, pady=4)
        self.status = tk.StringVar()
        ttk.Label(self.root, textvariable=self.status, wraplength=1000).pack(
            fill="x", padx=8, pady=4
        )
        board = ttk.LabelFrame(
            self.root, text="尚未标定时：更换为棋盘格采集，AprilTag 不能代替棋盘格"
        )
        board.pack(fill="x", padx=8, pady=4)
        for col, (name, label) in enumerate(
            (
                ("board_cols", "内角点列"),
                ("board_rows", "内角点行"),
                ("square_size_mm", "实测格长 mm"),
            )
        ):
            ttk.Label(board, text=label).grid(row=0, column=col * 2)
            self.values[name] = tk.StringVar(value=str(node.get_parameter(name).value))
            ttk.Entry(board, textvariable=self.values[name], width=8).grid(
                row=0, column=col * 2 + 1
            )
        ttk.Button(board, text="采集一张棋盘格", command=self.capture).grid(
            row=1, column=0, columnspan=3, pady=4
        )
        ttk.Button(board, text="计算内参并加载", command=self.calibrate).grid(
            row=1, column=3, columnspan=3, pady=4
        )
        self.notice = tk.StringVar(
            value="相机光学坐标：X 向右，Y 向下，Z 向前；距离到 Tag 中心。"
        )
        ttk.Label(self.root, textvariable=self.notice, wraplength=1000).pack(
            fill="x", padx=8, pady=4
        )

    def choose_file(self):
        path = filedialog.askopenfilename(
            title="选择真实相机标定结果", filetypes=[("YAML", "*.yaml *.yml")]
        )
        if path:
            self.values["calibration_file"].set(path)

    def apply(self):
        try:
            parameters = []
            for name in ("calibration_file", "tag_family", "tag_id", "tag_size_mm"):
                value = self.values[name].get()
                value = (
                    int(value)
                    if name == "tag_id"
                    else float(value) if name == "tag_size_mm" else value
                )
                parameters.append(Parameter(name, value=value))
            result = self.node.set_parameters_atomically(parameters)
            self.notice.set("测量设置已应用" if result.successful else result.reason)
        except ValueError as error:
            self.notice.set("设置格式错误：" + str(error))

    def board_settings(self):
        parameters = [
            Parameter("board_cols", value=int(self.values["board_cols"].get())),
            Parameter("board_rows", value=int(self.values["board_rows"].get())),
            Parameter(
                "square_size_mm", value=float(self.values["square_size_mm"].get())
            ),
        ]
        result = self.node.set_parameters_atomically(parameters)
        if not result.successful:
            raise ValueError(result.reason)

    def save_frame(self):
        try:
            self.notice.set(self.node.save_frame())
        except (ValueError, OSError, cv2.error) as error:
            self.notice.set(str(error))

    def capture(self):
        try:
            self.board_settings()
            self.notice.set(self.node.capture_sample())
        except (ValueError, OSError, cv2.error) as error:
            self.notice.set(str(error))

    def calibrate(self):
        try:
            self.board_settings()
            self.notice.set(self.node.start_calibration())
        except (ValueError, OSError) as error:
            self.notice.set(str(error))

    def refresh(self, image, status):
        if self.closed:
            return
        if image is not None:
            scale = min(960 / image.shape[1], 640 / image.shape[0], 1.0)
            display = cv2.resize(image, None, fx=scale, fy=scale)
            self.photo = ImageTk.PhotoImage(
                Image.fromarray(cv2.cvtColor(display, cv2.COLOR_BGR2RGB))
            )
            self.image.configure(image=self.photo)
        self.fps.set(
            f"相机接收：{status.get('receive_fps', 0.0):.1f} FPS    "
            f"检测处理：{status.get('process_fps', 0.0):.1f} FPS    "
            f"累计接收：{status.get('received_frames', 0)} 帧"
        )
        self.status.set(status["message"])
        if status["valid"]:
            x, y, z = status["xyz_mm"]
            self.pose.set(
                f'Tag {status["tag_id"]}：X={x:.1f}  Y={y:.1f}  Z={z:.1f} mm    直线距离={status["distance_mm"]:.1f} mm'
            )
        else:
            self.pose.set("尚无有效测量：— mm")
        self.root.update_idletasks()
        self.root.update()

    def close(self):
        self.closed = True
        self.root.destroy()
