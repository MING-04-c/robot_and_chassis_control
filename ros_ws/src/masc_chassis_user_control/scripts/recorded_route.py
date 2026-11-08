"""里程计路线记录及倒车 Pure Pursuit；米、弧度，不依赖 ROS 或相机像素。"""

import bisect
import json
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from time import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class RouteConfig:
    sample_distance: float = .02
    lookahead: float = .20
    max_speed: float = .10
    lookahead_speed_reference: float = .10
    min_lookahead: float = .08
    max_lookahead: float = .36
    max_lateral_accel: float = .025
    approach_distance: float = .10
    finish_distance: float = .30
    finish_straight_distance: float = .60
    finish_center_gain: float = .65
    finish_lateral_sign: float = 1.
    finish_heading_gain: float = .60
    finish_entry_heading: float = .40
    finish_entry_lateral: float = .65
    finish_exit_heading: float = .65
    finish_exit_lateral: float = .75
    finish_goal_lateral: float = .08
    finish_goal_heading: float = math.radians(6.)
    visual_strafe: bool = True
    visual_lateral_speed: float = .025
    visual_min_lateral_speed: float = .012
    visual_min_angular_speed: float = .02
    visual_lateral_gain: float = .5
    visual_image_lateral_gain: float = .05
    visual_goal_lateral_m: float = .02
    finish_entry_bend: float = .15
    finish_entry_yaw_rate: float = .06
    finish_center_tolerance: float = .02
    finish_speed: float = .03
    finish_extra_distance: float = .10
    max_curvature: float = 2.
    max_tracking_error: float = .15
    goal_tolerance: float = .02
    odom_timeout: float = .5

    def validate(self):
        limits = {'sample_distance': (.005, .05), 'lookahead': (.05, .6),
                  'max_speed': (.01, .20), 'lookahead_speed_reference': (.02, .3),
                  'min_lookahead': (.05, .6), 'max_lookahead': (.05, .8),
                  'max_lateral_accel': (.005, .2), 'approach_distance': (.05, .6),
                  'finish_distance': (.10, .60), 'finish_straight_distance': (.10, 1.),
                  'finish_center_gain': (.1, 1.), 'finish_heading_gain': (.1, 1.5),
                  'finish_entry_lateral': (.1, .7),
                  'finish_entry_heading': (.12, .45), 'finish_exit_heading': (.4, .7),
                  'finish_exit_lateral': (.5, .95),
                  'finish_goal_lateral': (.02, .2), 'finish_goal_heading': (.03, .2),
                  'visual_lateral_speed': (.005, .04), 'visual_lateral_gain': (.1, 1.),
                  'visual_min_lateral_speed': (.011, .025), 'visual_min_angular_speed': (.011, .04),
                  'visual_image_lateral_gain': (.01, .1), 'visual_goal_lateral_m': (.005, .05),
                  'finish_entry_bend': (.04, .2), 'finish_entry_yaw_rate': (.02, .1),
                  'finish_center_tolerance': (.005, .06), 'finish_speed': (.005, .04),
                  'finish_extra_distance': (0., .10),
                  'max_curvature': (.2, 5.), 'max_tracking_error': (.03, .3),
                  'goal_tolerance': (.005, .05), 'odom_timeout': (.1, 1.)}
        for name, (low, high) in limits.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'route_{name} must be {low}..{high}')

        if isinstance(self.finish_lateral_sign, bool) or self.finish_lateral_sign not in (-1., 1.):
            raise ValueError('route_finish_lateral_sign must be -1 or 1')
        if self.min_lookahead > self.max_lookahead:
            raise ValueError('route min lookahead must not exceed max lookahead')
        if type(self.visual_strafe) is not bool:
            raise ValueError('route_visual_strafe must be boolean')
        if self.visual_min_lateral_speed > self.visual_lateral_speed:
            raise ValueError('visual minimum lateral speed exceeds maximum')
        if not self.finish_goal_lateral < self.finish_entry_lateral < self.finish_exit_lateral:
            raise ValueError('visual lateral goal < entry < exit limits required')
        if self.finish_exit_heading-self.finish_entry_heading < .1:
            raise ValueError('visual heading exit limit must exceed entry limit by at least .1rad')
        if self.finish_straight_distance < self.finish_distance:
            raise ValueError('route_finish_straight_distance must be >= route_finish_distance')

    def visual_entry_ok(self, feedback):
        """仅判断近端线能否进入纠偏；最终到位另用更严格容差，不改变10cm搜索围栏。"""
        if not isinstance(feedback, dict):
            return False
        error, heading = feedback.get('error_normalized'), feedback.get('near_heading_rad')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in (error, heading)):
            return False
        return abs(error) <= self.finish_entry_lateral and abs(heading) <= self.finish_entry_heading

    def finish_alignment_ok(self, error, heading):
        """最终到位：半图宽归一化偏差及弧度朝向均合格；不改变途中恢复范围。"""
        return (math.isfinite(error) and math.isfinite(heading)
                and abs(error) <= self.finish_goal_lateral
                and abs(heading) <= self.finish_goal_heading)


class RouteRecorder:
    """输入连续里程计 (x,y,yaw)，稀疏记录实际行驶轨迹并原子写入 JSON。"""
    def __init__(self, config, frame, child):
        self.config, self.frame, self.child = config, frame, child
        self.points = []
        self.station_events = []
        self.latest = None
        self.context = {}
        self.samples, self.latest_sample = [], None
        self.fault = None
        self.finished = False

    def add(self, pose, stamp=None, velocity=None):
        self.latest = tuple(pose)
        self.latest_sample = dict(stamp_sec=stamp, velocity=list(velocity) if velocity is not None else None)
        if (not self.points or math.dist(pose[:2], self.points[-1][:2]) >= self.config.sample_distance
                or abs(wrap(pose[2]-self.points[-1][2])) >= .05):
            if len(self.points) >= 20000:
                self.fault = 'route_point_limit'
            else:
                self.points.append(tuple(pose))
                self.samples.append(self.latest_sample)

    def record_station(self, event):
        """保存站点事件供返程核对；接收时里程计位置仅作追踪，不是横线的标定坐标。"""
        self.station_events.append(dict(event))
        self.station_events = self.station_events[-1024:]

    def document(self):
        points = list(self.points)
        samples = list(self.samples)
        if self.latest is not None and (not points or self.latest != points[-1]):
            points.append(self.latest)
            samples.append(self.latest_sample)
        length = sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))
        return {'version': 1, 'valid': self.fault is None and length >= .1 and self.finished,
                'finished': self.finished,
                'reason': self.fault or ('recording_in_progress' if not self.finished else
                                         'ready' if length >= .1 else 'route_too_short'),
                'saved_at_unix': time(), 'frame_id': self.frame, 'child_frame_id': self.child,
                'length_m': length, 'points': points, 'point_samples': samples,
                'recording_config': dict(self.context), 'station_events': list(self.station_events)}

    def save(self, filename):
        """原子替换最近一次路线；新任务不足10cm也写明无效，避免误用旧路线。"""
        write_route_document((filename, self.document()))


def write_route_document(request):
    """写入已脱离记录器的快照，允许后台保存而不并发读取可变点列。"""
    filename, document = request
    path = Path(filename).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.'+path.name, delete=False) as stream:
            temporary = stream.name
            json.dump(document, stream, allow_nan=False, separators=(',', ':'))
        os.replace(temporary, path)
        return document
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def load_route(filename):
    """读取并检查路线；返回文档，缺失/损坏/过短/不连续时抛出 ValueError。"""
    path = Path(filename).expanduser()
    if path.stat().st_size > 4_000_000:
        raise ValueError('route_file_too_large')
    data = json.loads(path.read_text(encoding='utf-8'))
    if (not isinstance(data, dict) or data.get('version') != 1
            or data.get('valid') is not True or data.get('finished') is not True):
        raise ValueError('route_invalid_or_too_short')
    points = data.get('points')
    if not isinstance(points, list) or not 2 <= len(points) <= 20001:
        raise ValueError('route_invalid_points')
    for point in points:
        if (not isinstance(point, list) or len(point) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in point)):
            raise ValueError('route_nonfinite_pose')
    gaps = [math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:])]
    if sum(gaps) < .1 or max(gaps) > .25:
        raise ValueError('route_short_or_discontinuous')
    if not data.get('frame_id') or not data.get('child_frame_id'):
        raise ValueError('route_missing_frame')
    data['length_m'] = sum(gaps)
    return data


class ReverseRouteFollower:
    """将记录终点对齐倒车起点；按弧长前瞻车后路径，不追逐车前像素。"""
    def __init__(self, document, anchor, config):
        self.config = config
        end = document['points'][-1]
        rotation = anchor[2]-end[2]
        c, s = math.cos(rotation), math.sin(rotation)
        self.points = []
        for x, y, _ in reversed(document['points']):
            dx, dy = x-end[0], y-end[1]
            point = (anchor[0]+c*dx-s*dy, anchor[1]+s*dx+c*dy)
            if not self.points or math.dist(point, self.points[-1]) > 1e-6:
                self.points.append(point)
        if len(self.points) < 2:
            raise ValueError('route_too_short')
        self.arc = [0.]
        for a, b in zip(self.points, self.points[1:]):
            self.arc.append(self.arc[-1]+math.dist(a, b))
        self.progress = self.travel = 0.
        self.previous = tuple(anchor[:2])
        self.arrived = False
        self.diagnostics = {}

    def at(self, distance):
        index = max(0, min(len(self.points)-2, bisect.bisect_right(self.arc, distance)-1))
        fraction = max(0., min(1., (distance-self.arc[index])/(self.arc[index+1]-self.arc[index])))
        a, b = self.points[index:index+2]
        return a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1])

    @staticmethod
    def body_curvature(a, b, c):
        """三点弧线曲率；记录点按倒车行进方向排列，换成车体带符号曲率需反号。"""
        ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
        if min(ab, bc, ac) < .001:
            return 0.
        cross = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        return -2*cross/(ab*bc*ac)

    def reference_curvature(self):
        """只读记录路径形状，不叠加里程计横向偏差；用于视觉阶段小弯前馈。"""
        end = self.arc[-1]
        middle = max(.05, min(end-.05, self.progress))
        curvature = self.body_curvature(self.at(middle-.05), self.at(middle), self.at(middle+.05))
        cap = self.config.max_curvature
        # 超过记录起点的找线区域不预测未知曲线，末10cm逐步减小前馈。
        return max(-cap, min(cap, curvature))*min(1., max(0., (end-self.progress)/.1))

    def remaining_is_gentle(self):
        """按10cm实测路径窗口检查小弯；拒绝急弯/折返，不以当前单帧直线代替后方路线。"""
        end = self.arc[-1]
        start = min(self.progress, max(0., end-.2))
        length = end-start
        chord = math.dist(self.at(start), self.points[-1])
        if chord < .05 or length > 1.25*chord:
            return False
        count = max(2, math.ceil(length/.05))
        samples = [self.at(start+length*i/count) for i in range(count+1)]
        return all(abs(self.body_curvature(a, b, c)) <= self.config.max_curvature
                   for a, b, c in zip(samples, samples[1:], samples[2:]))

    def remaining_is_straight(self):
        """剩余实际路线每5cm采样；允许小噪声，拒绝弯道及折返，仅用于提前视觉接管。"""
        start, end = self.at(self.progress), self.points[-1]
        dx, dy = end[0]-start[0], end[1]-start[1]
        length = math.hypot(dx, dy)
        if length < .1:
            return False
        remaining = self.arc[-1]-self.progress
        if remaining > 1.1*length:
            return False
        count = max(2, math.ceil(remaining/.05))
        samples = [self.at(self.progress+remaining*i/count) for i in range(count+1)]
        for x, y in samples:
            if abs((x-start[0])*dy-(y-start[1])*dx)/length > .025:
                return False
        for a, b in zip(samples, samples[1:]):
            sx, sy = b[0]-a[0], b[1]-a[1]
            if math.hypot(sx, sy) < .001 or abs(math.atan2(sx*dy-sy*dx, sx*dx+sy*dy)) > .20:
                return False
        return True

    def command(self, pose, speed):
        """输入当前里程计位姿与速度绝对值；输出 (v,w,state)，异常时零速。"""
        if self.arrived:
            return 0., 0., 'reverse_route_complete'
        self.travel += math.dist(pose[:2], self.previous)
        self.previous = tuple(pose[:2])
        # 仅在当前弧长附近寻找投影，且进度不能跳过实际行驶距离；防交叉线跳段。
        lookahead = self.adaptive_lookahead(speed)
        upper = min(self.arc[-1], self.progress+max(.3, 2*lookahead), self.travel+.05)
        first = max(0, bisect.bisect_right(self.arc, max(0., self.progress-.03))-1)
        last = min(len(self.points)-1, bisect.bisect_right(self.arc, upper))
        best = (math.inf, self.progress)
        for index in range(first, last):
            a, b = self.points[index:index+2]
            dx, dy = b[0]-a[0], b[1]-a[1]
            fraction = max(0., min(1., ((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/(dx*dx+dy*dy)))
            along = self.arc[index]+fraction*(self.arc[index+1]-self.arc[index])
            if along > upper:
                continue
            distance = math.hypot(pose[0]-a[0]-fraction*dx, pose[1]-a[1]-fraction*dy)
            if distance < best[0]-1e-9:
                best = distance, along
        error, projection = best
        self.progress = max(self.progress, projection)
        remaining = self.arc[-1]-self.progress
        goal_distance = math.dist(pose[:2], self.points[-1])
        self.diagnostics = {'remaining_m': remaining, 'tracking_error_m': error if math.isfinite(error) else None,
                            'progress_m': self.progress, 'goal_distance_m': goal_distance}
        if error > self.config.max_tracking_error:
            return 0., 0., 'reverse_route_deviation'
        if remaining <= self.config.goal_tolerance and goal_distance <= self.config.goal_tolerance:
            self.arrived = True
            return 0., 0., 'reverse_route_complete'
        target = self.at(min(self.arc[-1], self.progress+lookahead))
        dx, dy = target[0]-pose[0], target[1]-pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        bx, by = c*dx+s*dy, -s*dx+c*dy
        self.diagnostics['target_body_xy_m'] = (bx, by)
        self.diagnostics['target_rear_angle_rad'] = math.atan2(by, -bx)
        if bx >= -.001 or abs(math.atan2(by, -bx)) > .9:
            return 0., 0., 'reverse_target_not_behind'
        requested_curvature = 2*by/(bx*bx+by*by)
        curvature = max(-self.config.max_curvature, min(self.config.max_curvature,
                                                       requested_curvature))
        # Nav2 RPP-style regulation: use speed-scaled lookahead and cap lateral
        # acceleration in bends, while keeping terminal approach deceleration separate.
        cruise = min(abs(speed), self.config.max_speed)
        curvature_speed = (math.sqrt(self.config.max_lateral_accel/abs(curvature))
                           if abs(curvature) > 1e-6 else self.config.max_speed)
        cruise = min(cruise, curvature_speed)
        velocity = -min(cruise, max(.005, cruise*remaining/self.config.approach_distance))
        self.diagnostics.update(curvature_per_m=curvature, lookahead_m=lookahead,
                                requested_curvature_per_m=requested_curvature,
                                curvature_limit_per_m=self.config.max_curvature,
                                curvature_limited=abs(requested_curvature) > self.config.max_curvature,
                                curvature_speed_limit_mps=curvature_speed,
                                cruise_speed_mps=cruise)
        return velocity, velocity*curvature, 'reverse_route_tracking'

    def adaptive_lookahead(self, speed):
        """Scale the configured RPP lookahead with speed, then apply hard bounds."""
        reference = max(.001, self.config.lookahead_speed_reference)
        scaled = self.config.lookahead*abs(float(speed))/reference
        return max(self.config.min_lookahead, min(self.config.max_lookahead, scaled))


class StandardRouteFollower:
    """标准路线前进Pure Pursuit；起点需由操作员放回同一实物基准并通过视觉对线。"""
    def __init__(self, document, anchor, config):
        transformed = dict(document, points=[[x, y, wrap(yaw+math.pi)]
                           for x, y, yaw in reversed(document['points'])])
        self.tracker = ReverseRouteFollower(transformed,
            (anchor[0], anchor[1], wrap(anchor[2]+math.pi)), config)

    @property
    def diagnostics(self):
        result = dict(self.tracker.diagnostics)
        if 'curvature_per_m' in result:
            result['curvature_per_m'] = -result['curvature_per_m']
        if 'target_body_xy_m' in result:
            result['target_body_xy_m'] = tuple(-v for v in result['target_body_xy_m'])
        return result

    def command(self, pose, speed):
        v, w, state = self.tracker.command((pose[0], pose[1], wrap(pose[2]+math.pi)), speed)
        return -v, w, ('standard_target_not_ahead' if state == 'reverse_target_not_behind'
                       else state.replace('reverse_', 'standard_'))


class ReverseFinishGuard:
    """最后直线视觉找点的里程计行程围栏，不用里程计到位冒充找到横线。"""
    def __init__(self, tracker, pose, config):
        self.config = config
        self.tracker = tracker
        self.goal = tracker.points[-1]
        before = tracker.at(max(0., tracker.arc[-1]-.10))
        dx, dy = self.goal[0]-before[0], self.goal[1]-before[1]
        length = math.hypot(dx, dy)
        if length < .01:
            raise ValueError('reverse_finish_route_not_straight')
        self.tangent = dx/length, dy/length
        self.previous = tuple(pose[:2])
        self.travel = 0.
        self.travel_limit = tracker.diagnostics['remaining_m']+config.finish_extra_distance+.05
        self.fault = None
        self.arrived = False
        self.curvature = 0.
        self.diagnostics = {}

    def check(self, pose):
        if self.arrived:
            return 'reverse_route_complete'
        self.travel += math.dist(pose[:2], self.previous)
        self.previous = tuple(pose[:2])
        dx, dy = pose[0]-self.goal[0], pose[1]-self.goal[1]
        tx, ty = self.tangent
        beyond = dx*tx+dy*ty
        lateral = abs(-dx*ty+dy*tx)
        if beyond <= 0.:
            # 小弯内使用剩余记录路径走廊，不能把弯线到终点切线的距离当成偏离。
            points = self.tracker.points
            first = max(0, bisect.bisect_right(self.tracker.arc, max(0., self.tracker.progress-.1))-1)
            distances = []
            for a, b in zip(points[first:], points[first+1:]):
                sx, sy = b[0]-a[0], b[1]-a[1]
                fraction = max(0., min(1., ((pose[0]-a[0])*sx+(pose[1]-a[1])*sy)/(sx*sx+sy*sy)))
                distances.append(math.hypot(pose[0]-a[0]-fraction*sx, pose[1]-a[1]-fraction*sy))
            lateral = min(distances) if distances else math.dist(pose[:2], self.goal)
        extra = math.hypot(dx, dy) if beyond > 0. else 0.
        # 围栏刷新不能抹掉上一条实际运动指令；停车后仍需据此检查纠偏方向。
        self.diagnostics.update(extra_distance_m=extra, visual_travel_m=self.travel,
                                extra_distance_limit_m=self.config.finish_extra_distance,
                                lateral_error_m=lateral, control_terms_valid=False)
        if extra > self.config.finish_extra_distance+1e-6 or self.travel > self.travel_limit:
            self.fault = 'reverse_finish_search_limit'
        elif lateral > self.config.max_tracking_error:
            self.fault = 'reverse_finish_deviation'
        return self.fault
