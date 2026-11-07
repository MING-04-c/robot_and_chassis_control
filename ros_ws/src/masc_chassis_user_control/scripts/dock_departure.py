"""充电桩前进离站：只豁免桩充电信号，距离/时间耗尽后锁停。"""
import math


class DockDeparture:
    def __init__(self, max_distance=.30, timeout=25.):
        if not (.01 <= max_distance <= .50 and 1. <= timeout <= 60.):
            raise ValueError('离桩最大距离需为0.01..0.50m，超时需为1..60s')
        self.max_distance, self.timeout = max_distance, timeout
        self.reset()

    def reset(self):
        self.state = 'idle'
        self.started = self.last_pose = None
        self.distance = 0.
        self.clear_frames = 0
        self.last_stamp = None

    def observe(self, charging, stamp):
        """只计算不同CAN帧，连续三帧脱离充电才结束离桩。"""
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        self.clear_frames = 0 if charging else self.clear_frames+1
        if self.state == 'active' and self.clear_frames >= 3:
            self.state = 'complete'

    def advance(self, now, pose, charging):
        """使能后计时，使用实际里程累计；包括暂停时段，故障不自动重试。"""
        if self.state == 'idle':
            if not charging:
                return
            self.state, self.started, self.last_pose = 'active', now, tuple(pose[:2])
        if self.state != 'active':
            return
        self.distance += math.hypot(pose[0]-self.last_pose[0], pose[1]-self.last_pose[1])
        self.last_pose = tuple(pose[:2])
        if now-self.started >= self.timeout:
            self.state = 'timeout'
        elif self.distance >= self.max_distance:
            self.state = 'distance_exceeded'

    @property
    def fault(self):
        return 'dock_departure_'+self.state if self.state in ('timeout', 'distance_exceeded') else None

    def snapshot(self):
        return dict(state=self.state, distance_m=self.distance,
                    max_distance_m=self.max_distance, timeout_sec=self.timeout)
