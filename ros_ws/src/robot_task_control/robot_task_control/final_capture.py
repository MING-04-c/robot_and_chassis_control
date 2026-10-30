"""任务末尾按需启动相机，等待画面稳定，顺序保存及发送；不控制机器人。"""

import argparse
import hashlib
from contextlib import contextmanager
import importlib.util
import ipaddress
import json
import math
from pathlib import Path
import signal
import shutil
import struct
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
import yaml


def load_config(path):
    """读取受限采集配置；输入 YAML 路径，返回已检查的字典。"""
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("拍照配置必须是 mapping")
    for key in ('receiver_ip', 'hk_bind_ip'):
        ipaddress.IPv4Address(config[key])
    for key in ('hk_port', 'rgbd_port'):
        if type(config[key]) is not int or not 1 <= config[key] <= 65535:
            raise ValueError(key + " 必须是有效端口")
    for key in ('robot_settle_sec', 'camera_warmup_sec', 'transfer_timeout_sec',
                'stability_timeout_sec', 'frame_difference_threshold', 'hk_frame_rate'):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(key + " 必须为有限正数")
    if config['stability_timeout_sec'] <= config['camera_warmup_sec']:
        raise ValueError("稳定检查超时必须大于相机预热时间")
    if type(config['stable_frames']) is not int or not 3 <= config['stable_frames'] <= 100:
        raise ValueError("stable_frames 必须为3～100")
    if type(config['jpeg_quality']) is not int or not 1 <= config['jpeg_quality'] <= 100:
        raise ValueError("jpeg_quality 必须为1～100")
    if type(config['pause_line_camera']) is not bool:
        raise ValueError("pause_line_camera 必须为 bool")
    return config


class StableFrames:
    """对新帧的小尺寸灰度图检查连续变化；输入图像/时间，输出是否稳定。"""
    def __init__(self, config):
        self.config = config
        self.first_time = None
        self.previous = None
        self.count = 0
        self.difference = None
        self.last_time = None

    def observe(self, frame, now):
        small = cv2.cvtColor(cv2.resize(frame, (160, 120)), cv2.COLOR_BGR2GRAY)
        if self.first_time is None:
            self.first_time = now
        if self.previous is not None:
            self.difference = float(np.mean(cv2.absdiff(small, self.previous)))
        if self.last_time is not None and now-self.last_time > .5:
            self.count = 0
            self.difference = None
        self.last_time = now
        self.previous = small
        if now-self.first_time < self.config['camera_warmup_sec']:
            self.count = 0
        elif self.difference is not None and self.difference <= self.config['frame_difference_threshold']:
            self.count += 1
        else:
            self.count = 0
        return self.count >= self.config['stable_frames']


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def depth_module(name):
    """复用已安装的主机兼容保存/打包代码，不复制一套数据格式。"""
    directory = Path(get_package_prefix('geminipro_camer_calibration')) / 'lib/geminipro_camer_calibration'
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
    spec = importlib.util.spec_from_file_location(name, directory / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@contextmanager
def camera_process(command):
    """只管理本任务创建的直接驱动进程，正常/异常退出均释放设备。"""
    process = subprocess.Popen(command)
    try:
        yield process
    finally:
        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        if process.returncode not in (0, -signal.SIGINT):
            print(f"相机进程退出码：{process.returncode}", flush=True)


class CaptureSession(Node):
    def __init__(self, config, output, storage_root=None):
        super().__init__('task_final_capture')
        self.config, self.output = config, output
        self.storage_root = Path(storage_root) if storage_root else output
        self.bridge = CvBridge()
        self.report = {'schema': 'task_final_capture_v1', 'session_id': output.name,
                       'state': 'starting', 'hk': None, 'rgbd': None}
        self.write_state('starting')

    def write_state(self, state, **fields):
        self.report.update(state=state, **fields)
        path = self.output / 'status.json'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(path)
        from capture_storage import atomic_json
        atomic_json(self.storage_root/'.capture_status.json', self.report)
        self.get_logger().info('任务末尾采集：' + state)

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
        if not rclpy.ok():
            raise RuntimeError('采集已取消')

    def pause_line_camera(self):
        if not self.config['pause_line_camera']:
            return
        client = self.create_client(SetBool, self.config['line_streaming_service'])
        try:
            optional = self.config.get('allow_no_line_camera', False)
            if not client.wait_for_service(timeout_sec=1 if optional else 5):
                if optional:
                    self.get_logger().info('没有循迹相机服务，按独立相机模式采集')
                    return
                raise RuntimeError('未找到循迹相机暂停服务，请重新编译 ruier_camer_diver')
            request = SetBool.Request()
            request.data = False
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5)
            if not future.done() or not future.result() or not future.result().success:
                raise RuntimeError('循迹相机暂停失败')
        finally:
            self.destroy_client(client)

    def stable_image(self, topic, process):
        monitor = StableFrames(self.config)
        latest = [None]
        stamp = [None]
        def on_image(message):
            key = (message.header.stamp.sec, message.header.stamp.nanosec)
            age = (self.get_clock().now().nanoseconds-key[0]*10**9-key[1])/1e9
            if key == stamp[0] or not 0 <= age <= 1.0:
                return
            stamp[0] = key
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            if monitor.observe(frame, time.monotonic()):
                latest[0] = message
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscription = self.create_subscription(Image, topic, on_image, qos)
        deadline = time.monotonic() + self.config['stability_timeout_sec']
        try:
            while time.monotonic() < deadline and rclpy.ok():
                if process.poll() is not None:
                    raise RuntimeError(f'相机提前退出：{process.returncode}')
                rclpy.spin_once(self, timeout_sec=.1)
                if latest[0] is not None:
                    return latest[0]
            raise RuntimeError(f'{topic} 画面未稳定或没有新帧，禁止用旧图拍照；最后变化={monitor.difference}')
        finally:
            self.destroy_subscription(subscription)

    def persist_hk(self):
        from capture_storage import save_hk, capture_label
        header = {**self.report['hk'], 'capture_index': 1, 'version': 1}
        saved = save_hk(self.storage_root, self.output.name, header, (self.output/'hk.jpg').read_bytes())
        self.report['capture_name'] = capture_label(self.storage_root, self.output.name)
        self.get_logger().info('照片目录：'+str(saved.parent))

    def persist_rgbd(self, folder):
        from capture_storage import save_rgbd
        save_rgbd(self.storage_root, self.output.name, folder, self.report.get('capture_name'))

    def capture(self):
        self.pause_line_camera()
        self.write_state('waiting_robot_settle')
        self.wait(self.config['robot_settle_sec'])
        # 防止重复占用已经运行的相机；不终止用户自行启动的驱动。
        names = {name for name, _ in self.get_node_names_and_namespaces()}
        if 'hik_camera_ros2_driver' in names or 'orbbec_camera_node' in names:
            raise RuntimeError('发现已启动的拍照相机，请关闭手动 HK/深度相机后再运行本任务')
        hk_params = self.config['hk_params'] or str(
            Path(get_package_share_directory('hik_camera_ros2_driver')) / 'config/camera_params.yaml'
        )
        self.write_state('hk_warming')
        with camera_process([executable('hik_camera_ros2_driver', 'hik_camera_ros2_driver_node'),
                             '--ros-args', '-r', '__node:=hik_camera_ros2_driver',
                             '--params-file', str(Path(hk_params).expanduser()),
                             '-p', f"acquisition_frame_rate:={self.config['hk_frame_rate']}"]) as process:
            message = self.stable_image(self.config['hk_image_topic'], process)
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.config['jpeg_quality']])
            if not ok:
                raise RuntimeError('HK JPEG 编码失败')
            photo = self.output / 'hk.jpg'
            photo.write_bytes(encoded.tobytes())
            self.report['hk'] = {'file': 'hk.jpg', 'width': message.width, 'height': message.height,
                                 'stamp_ns': message.header.stamp.sec*10**9+message.header.stamp.nanosec}
            del frame, encoded, message
        self.persist_hk()
        self.write_state('hk_saved_camera_closed')
        depth_params = self.config['depth_params'] or str(
            Path(get_package_share_directory('geminipro_camer_calibration')) / 'config/gemini_pro_capture.yaml'
        )
        namespace = self.config['depth_namespace'].strip('/')
        self.write_state('depth_warming')
        with camera_process([executable('orbbec_camera', 'orbbec_camera_node'), '--ros-args',
                             '-r', '__node:=orbbec_camera_node', '-r', '__ns:=/'+namespace,
                             '--params-file', str(Path(depth_params).expanduser())]) as process:
            self.stable_image('/'+namespace+'/color/image_raw', process)
            from orbbec_camera_msgs.srv import GetCaptureFrame
            client = self.create_client(GetCaptureFrame, '/'+namespace+'/capture_frame')
            try:
                if not client.wait_for_service(timeout_sec=5):
                    raise RuntimeError('SDK 拍照服务不可用')
                saver = depth_module('capture_v54')
                for attempt in range(1, 4):
                    future = client.call_async(GetCaptureFrame.Request())
                    rclpy.spin_until_future_complete(self, future, timeout_sec=26)
                    if not future.done() or future.result() is None:
                        raise RuntimeError('深度相机同帧拍照服务超时')
                    try:
                        folder = self.output/'rgbd'/('camera_v5_4_'+self.output.name)/f'attempt_{attempt:02d}'
                        saved = saver.save_capture(future.result(), folder)
                        break
                    except saver.RetryCapture:
                        if attempt == 3:
                            raise
                self.persist_rgbd(saved)
                self.report['rgbd'] = str(saved.relative_to(self.output))
                (self.output/'rgbd'/'dataset_summary.json').write_text(json.dumps({
                    'schema': 'geminipro_v54_dataset_v1', 'count': 1,
                    'task_run_id': self.output.name, 'capture_name': self.report.get('capture_name'),
                    'groups': [{'directory': str(saved.relative_to(self.output/'rgbd'))}],
                }, ensure_ascii=False, indent=2), encoding='utf-8')
            finally:
                self.destroy_client(client)
        self.write_state('all_cameras_closed')

    def transfer(self):
        # 使用原来的两条独立 TCP 通道；只在采集完成后占用编码和网络资源。
        from industrial_pc_hk_transfer import ImageTcpServer, PhotoQueue, MAGIC
        frames = PhotoQueue(capacity=1)
        payload = (self.output/'hk.jpg').read_bytes()
        header = dict(version=1, type='image', sequence=1, encoding='jpeg',
                      payload_sha256=hashlib.sha256(payload).hexdigest(),
                      source_encoding='bgr8', payload_length=len(payload),
                      width=self.report['hk']['width'], height=self.report['hk']['height'],
                      stamp_ns=self.report['hk']['stamp_ns'], frame_id='camera_optical_frame',
                      topic=self.config['hk_image_topic'], transfer_session_id=self.output.name,
                      capture_set_id=self.output.name, capture_index=1, grouping='task_final_one_hk')
        if self.report.get('capture_name'):
            header['capture_name'] = self.report['capture_name']
        body = json.dumps(header, separators=(',', ':')).encode('utf-8')
        frames.publish(1, MAGIC+struct.pack('!I', len(body))+body+payload)
        del payload, body
        self.write_state('sending_hk')
        with ImageTcpServer((self.config['hk_bind_ip'], self.config['hk_port']), frames,
                            self.config['receiver_ip']) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                deadline = time.monotonic()+self.config['transfer_timeout_sec']
                while not frames.has_space():
                    if time.monotonic() >= deadline:
                        raise RuntimeError('HK TCP 发送超时；本地照片保留，请确认主机 listen_robot.py 已运行、9002未被占用')
                    self.wait(.1)
            finally:
                frames.close()
                server.shutdown()
                thread.join(timeout=2)
        self.report['hk_tcp'] = 'written_to_socket_no_disk_ack'
        self.write_state('sending_rgbd')
        transfer = depth_module('capture_transfer')
        archive = self.output/'rgbd.zip'
        transfer.pack(self.output/'rgbd', archive)
        deadline = time.monotonic()+self.config['transfer_timeout_sec']
        while True:
            try:
                transfer.send(archive, self.config['receiver_ip'], self.config['rgbd_port'])
                break
            except (OSError, EOFError) as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError('RGB-D TCP发送失败；本地数据保留：'+str(error)) from error
                self.wait(1.)
        self.report['rgbd_tcp'] = 'host_ack_received'
        self.write_state('complete')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--allow-no-line-camera', action='store_true',
                        help='独立采集时允许未启动循迹相机；存在该服务时仍暂停采流')
    args = parser.parse_args()
    if not args.run_id or any(ch not in '0123456789abcdef_-' for ch in args.run_id):
        parser.error('run-id 必须为安全的会话标识')
    config = load_config(args.config)
    config['allow_no_line_camera'] = args.allow_no_line_camera
    cv2.setNumThreads(1)  # 缩略图/编码使用单线程，给机器人控制留CPU。
    storage_root = Path(config['output_dir']).expanduser()
    output = storage_root/'.pending'/args.run_id
    output.mkdir(parents=True, exist_ok=False)
    # 父任务停止时允许 finally 清理本进程创建的相机。
    rclpy.init()
    def terminate(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, terminate)
    node = CaptureSession(config, output, storage_root)
    try:
        node.capture()
        node.transfer()
        try:
            shutil.rmtree(output)  # 成功后仅保留三图与集中索引。
        except OSError as error:
            node.get_logger().warning('图像已发送，临时文件未能全部清理：'+str(error))
    except (Exception, KeyboardInterrupt) as error:
        node.write_state('failed', error=str(error) or '采集取消')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
