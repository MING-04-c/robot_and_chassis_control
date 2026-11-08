#!/usr/bin/env python3
"""启动时一次性执行 dashboard 上电、释放抱闸并确认 RUNNING，不发送运动目标。"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger
from eli_common_interface.srv import GetRobotMode, GetSafetyMode
from eli_common_interface.msg import RobotMode, SafetyMode


class DashboardStartup(Node):
    """非阻塞服务状态机；每次只保留一个请求，拒绝时不重试上电/解锁。"""
    def __init__(self):
        super().__init__('elite_auto_power_on')
        timeout = float(self.declare_parameter('startup_timeout_sec', 240.).value)
        self.request_timeout = float(self.declare_parameter('request_timeout_sec', 10.).value)
        self.command_timeout = float(self.declare_parameter('command_timeout_sec', 120.).value)
        self.interval = float(self.declare_parameter('poll_interval_sec', .25).value)
        if (not all(math.isfinite(v) and v > 0 for v in (timeout, self.request_timeout, self.command_timeout, self.interval))
                or timeout > 300 or self.interval > 1.):
            raise ValueError('invalid dashboard startup timeouts')
        self.dashboard_clients = {
            'safety': self.create_client(GetSafetyMode, '/dashboard_client/get_safety_mode'),
            'mode': self.create_client(GetRobotMode, '/dashboard_client/robot_mode'),
            'power': self.create_client(Trigger, '/dashboard_client/power_on'),
            'brake': self.create_client(Trigger, '/dashboard_client/brake_release'),
        }
        self.deadline = time.monotonic()+timeout
        self.next_check = 0.
        self.pending = self.pending_kind = None
        self.request_deadline = 0.
        self.next_wait_log = 0.
        self.power_sent = self.brake_sent = False
        self.phase, self.result = 'services', None
        self.create_timer(.05, self.tick)
        self.get_logger().info('等待 dashboard 服务；启动顺序：上电 → 待机 → 释放抱闸 → RUNNING')

    def finish(self, success, message):
        self.result = 0 if success else 1
        (self.get_logger().info if success else self.get_logger().error)(message)

    def poll_again(self):
        self.phase = 'safety'
        self.next_check = time.monotonic()+self.interval

    def handle_response(self, kind, response):
        if not response.success:
            self.finish(False, f'dashboard {kind} 拒绝：{response.message}；取消控制启动')
            return
        if kind == 'safety':
            if response.mode.mode == SafetyMode.UNKNOWN:
                self.poll_again()
            elif response.mode.mode not in (SafetyMode.NORMAL, SafetyMode.REDUCED):
                self.finish(False, f'机器人安全模式={response.mode.mode}；请处理安全状态，不自动清除急停或保护停止')
            else:
                self.phase = 'mode'
        elif kind == 'mode':
            mode = response.mode.mode
            if mode == RobotMode.RUNNING:
                self.finish(True, '机器人已上电、抱闸已释放且处于 RUNNING，继续启动 ROS 控制器')
            elif mode == RobotMode.POWER_OFF and not self.power_sent:
                self.phase = 'power'
            elif mode == RobotMode.IDLE and not self.brake_sent:
                self.phase = 'brake'
            elif mode in (RobotMode.POWER_OFF, RobotMode.POWER_ON, RobotMode.IDLE,
                          RobotMode.BOOTING, RobotMode.DISCONNECTED, RobotMode.NO_CONTROLLER, RobotMode.UNKNOWN):
                self.poll_again()
            else:
                self.finish(False, f'机器人模式={mode}，不适合自动上电；取消控制启动')
        elif kind == 'power':
            self.power_sent = True
            self.get_logger().info('power_on 成功；等待 IDLE 后释放抱闸')
            self.poll_again()
        elif kind == 'brake':
            self.brake_sent = True
            self.get_logger().info('brake_release 成功；等待机器人进入 RUNNING')
            self.poll_again()

    def tick(self):
        if self.result is not None:
            return
        now = time.monotonic()
        if now >= self.deadline:
            self.finish(False, f'自动上电总等待超时，phase={self.phase}；请检查连接、远程模式及机器人状态')
            return
        if self.pending is not None:
            if not self.pending.done():
                if now >= self.request_deadline:
                    self.pending.cancel()
                    self.finish(False, f'dashboard {self.pending_kind} 服务响应超时；不重复发送命令')
                elif self.pending_kind in ('power', 'brake') and now >= self.next_wait_log:
                    self.get_logger().info(f'仍在等待 {self.pending_kind} 服务完成上电/抱闸动作；不会重复发送命令')
                    self.next_wait_log = now+10.
                return
            future, kind = self.pending, self.pending_kind
            self.pending = self.pending_kind = None
            try:
                self.handle_response(kind, future.result())
            except Exception as error:
                self.finish(False, f'dashboard {kind} 请求失败：{error}')
            return
        if self.phase == 'services':
            if not all(client.service_is_ready() for client in self.dashboard_clients.values()):
                return
            self.phase = 'safety'
        if now < self.next_check:
            return
        kind = self.phase
        request = {'safety': GetSafetyMode.Request, 'mode': GetRobotMode.Request,
                   'power': Trigger.Request, 'brake': Trigger.Request}[kind]()
        self.pending_kind = kind
        self.pending = self.dashboard_clients[kind].call_async(request)
        # SDK 的 powerOn/brakeRelease 会等到 IDLE/RUNNING 才返回，不能套用状态查询的短超时。
        wait = self.command_timeout if kind in ('power', 'brake') else self.request_timeout
        self.request_deadline = now+wait
        self.next_wait_log = now+10.
        if kind in ('power', 'brake'):
            self.get_logger().info(f'已发送 {kind}；最多等待 {wait:g} 秒，完成后再执行下一步')


def main():
    rclpy.init()
    node = DashboardStartup()
    result = 1
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=.1)
        result = node.result if node.result is not None else 130
    except (KeyboardInterrupt, ExternalShutdownException):
        result = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
