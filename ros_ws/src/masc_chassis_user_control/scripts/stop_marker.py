"""垂直于主线的停止横线识别，以及居中停车/等待新消息继续的状态机。"""
import math
from statistics import median, fmean
from time import monotonic

import cv2
import numpy as np


def travel_image_sign(config):
    """相机安装方向不变；倒车时只有横线接近/越线及站点延续方向反转。"""
    front = 1. if config.image_forward_direction == 'down' else -1.
    return -front if config.drive_direction == 'reverse' else front


def detect_stop_markers(mask, points, config, reference=None, diagnostics=None):
    """输入同尺寸单通道蓝线掩膜、实测中线点；输出横线交点及后续主线类别。"""
    diagnostics = {} if diagnostics is None else diagnostics
    diagnostics.clear()
    diagnostics.update(hough_segments=0, joined_segments=0, accepted=0, rejected={})
    def reject(reason):
        rejected = diagnostics['rejected']
        rejected[reason] = rejected.get(reason, 0)+1
    if mask is None or mask.ndim != 2 or len(points) < 3:
        reject('missing_mainline')
        return []
    height, width = mask.shape
    if any(not math.isfinite(v) for point in points for v in point):
        return []
    points = sorted(((float(x), float(y)) for x, y in points), key=lambda p: p[1])
    path_x, path_y = [p[0] for p in points], [p[1] for p in points]
    forward = travel_image_sign(config)
    binary = np.uint8(mask > 0) * 255
    edges = cv2.Canny(binary, 40, 100)
    # 保留长线候选，避免短横臂两两合并占用控制回调时间。
    lines = cv2.HoughLinesP(edges, 1, math.pi/180, 20,
                           minLineLength=max(20, int(width*config.stop_marker_min_span_ratio)),
                           maxLineGap=max(5, int(width*.05)))
    proposals = [] if lines is None else sorted(
        lines[:, 0], key=lambda p: math.hypot(p[2]-p[0], p[3]-p[1]), reverse=True)[:40]
    diagnostics['hough_segments'] = len(proposals)
    if not proposals:
        reject('no_bar_segment')
        return []
    found = []
    for x1, y1, x2, y2 in proposals:
        dx, dy = float(x2-x1), float(y2-y1)
        length = math.hypot(dx, dy)
        middle_y = (y1+y2) / 2.0
        local = [(x, y) for x, y in points if abs(y-middle_y) <= 0.14 * height]
        slopes = [(a[0]-b[0])/(a[1]-b[1]) for i, a in enumerate(local)
                  for b in local[i+1:] if abs(a[1]-b[1]) >= 3]
        if not slopes or len(local) < 3 or max(y for _, y in local)-min(y for _, y in local) < .04*height:
            reject('mainline_support')
            continue
        slope = median(slopes)
        intercept = median(x-slope*y for x, y in local)
        if (reference is not None and 'main_slope' in reference
                and abs(middle_y-reference['y']) < .08*height):
            # 停车后相机朝向不变，固定主线参考轴；横线位置仍每帧从真实像素重测。
            slope = reference['main_slope']
            intercept = reference['x']-slope*reference['y']
        if abs(slope*dx+dy) / (math.hypot(slope, 1)*length) > math.sin(config.stop_marker_angle_tolerance_rad):
            reject('bar_angle')
            continue
        denominator = dx-slope*dy
        if abs(denominator) < 1e-6:
            continue
        fraction = (slope*y1+intercept-x1)/denominator
        arm = width*config.stop_marker_min_arm_ratio
        if min(fraction, 1-fraction)*length < arm:
            reject('two_sided_arms')
            continue  # 必须两侧都有横线，不能把普通 L 形弯道当站点。
        cx, cy = x1+fraction*dx, y1+fraction*dy
        if not (0 <= cx < width and 0 <= cy < height):
            continue
        ux, uy = dx/length, dy/length
        nx, ny = -uy, ux
        # 用两侧横线的实体像素确定厚度中心，而不是把 Canny 的某条边当停车位置。
        radius = int(length/2 + .06*height)
        left, right = max(0, int(cx-radius)), min(width, int(cx+radius+1))
        top, bottom = max(0, int(cy-radius)), min(height, int(cy+radius+1))
        ys, xs = np.nonzero(binary[top:bottom, left:right])
        px, py = xs+left-cx, ys+top-cy
        along, normal = px*ux+py*uy, px*nx+py*ny
        wing = ((np.abs(along) >= arm) & (np.abs(along) <= length/2)
                & (np.abs(normal) <= .04*height))
        if np.count_nonzero(wing & (along < 0)) < 8 or np.count_nonzero(wing & (along > 0)) < 8:
            reject('wing_pixels')
            continue
        offsets = normal[wing]
        if np.percentile(offsets, 95)-np.percentile(offsets, 5) > .06*height:
            reject('bar_thickness')
            continue
        offset = float(np.median(offsets))
        cx, cy = cx+offset*nx, cy+offset*ny
        # 用两侧实体像素按列估计横线中心轴，避免挑中上/下边缘或不同短边而改变交点。
        if abs(dx) > .3*length:
            wing_x, wing_y = (xs+left)[wing], (ys+top)[wing]
            samples = []
            bin_width = max(3, int(.01*width))
            for x in range(int(wing_x.min()), int(wing_x.max())+1, bin_width):
                selected = (wing_x >= x) & (wing_x < x+bin_width)
                if np.count_nonzero(selected) >= 8:
                    samples.append((float(np.median(wing_x[selected])),
                                    float(np.median(wing_y[selected]))))
            bar_slopes = [(b[1]-a[1])/(b[0]-a[0]) for i, a in enumerate(samples)
                          for b in samples[i+1:] if abs(b[0]-a[0]) >= 2*bin_width]
            if not bar_slopes:
                continue
            bar_slope = median(bar_slopes)
            bar_intercept = median(y-bar_slope*x for x, y in samples)
            divisor = 1-bar_slope*slope
            if abs(divisor) < 1e-6:
                continue
            cy = (bar_intercept+bar_slope*intercept)/divisor
            cx = intercept+slope*cy
        else:
            correction = (slope*cy+intercept-cx)/(ux-slope*uy)
            cx, cy = cx+correction*ux, cy+correction*uy
        if not (0 <= cx < width and 0 <= cy < height):
            continue

        def supported_rows(direction, distance):
            hits = []
            for delta in np.arange(.035*height, distance*height, 3):
                y = int(round(cy+direction*delta))
                if not 0 <= y < height:
                    break
                x = int(round(float(np.interp(y, path_y, path_x))))
                # 超出实测范围时只搜索局部切线走廊，不把预测像素写入掩膜。
                if y < points[0][1] or y > points[-1][1]:
                    x = int(round(slope*y+intercept))
                half = max(3, int(.02*width))
                hits.append(bool(np.any(binary[y, max(0, x-half):min(width, x+half+1)])))
            return hits

        incoming = supported_rows(-forward, .14)
        if len(incoming) < 8 or sum(incoming)/len(incoming) < .65:
            reject('incoming_mainline')
            continue
        outgoing = supported_rows(forward, 1.0)
        longest = run = 0
        for hit in outgoing:
            run = run+1 if hit else 0
            longest = max(longest, run)
        if longest*3 >= config.stop_marker_continuation_ratio*height:
            kind = 'intermediate'
        elif len(outgoing)*3 >= .18*height and sum(outgoing)*3 <= .02*height:
            kind = 'final'
        else:
            kind = 'unknown'  # 横线还在画面边缘或主线证据含糊，不猜终点。
        if any(abs(cy-m['y']) < .035*height and abs(cx-m['x']) < .06*width for m in found):
            continue
        found.append({'x': float(cx), 'y': float(cy), 'kind': kind,
                      'span': length, 'angle_rad': math.atan2(dy, dx),
                      'main_slope': float(slope)})
    diagnostics['accepted'] = len(found)
    return found


class StationLedger:
    """一次任务内的横线身份及事件；重复观测不重复计数，不把漏检当作通过。"""
    def __init__(self):
        self.counts = dict(encountered=0, ignored=0, arrived=0, resumed=0, passed=0, uncertain=0)
        self.stations = []
        self.pending = []

    def encounter(self, marker, action='detected'):
        self.counts['encountered'] += 1
        identity = self.counts['encountered']
        self.stations.append(dict(id=identity, kind=marker['kind'], actions=[]))
        self.stations = self.stations[-128:]
        self.record(action, identity, marker)
        return identity

    def record(self, action, identity, marker=None, basis=None):
        station = next((row for row in self.stations if row['id'] == identity), None)
        if station is None or action in station['actions']:
            return
        station['actions'].append(action)
        station['state'] = action
        if marker is not None:
            station['kind'] = marker['kind']
        counter = {'ignored_departure': 'ignored', 'arrived': 'arrived', 'resumed': 'resumed',
                   'passed': 'passed'}.get(action)
        if action == 'released_unseen' and 'passed' not in station['actions']:
            counter = 'uncertain'
        if counter:
            self.counts[counter] += 1
        self.pending.append(dict(id=identity, action=action, kind=station['kind'],
                                 image_y=marker['y'] if marker is not None else None, basis=basis,
                                 counts=dict(self.counts)))

    def snapshot(self):
        return [dict(row, actions=list(row['actions'])) for row in self.stations]


class StopMarkerGate:
    """对同一横线的新帧位置滚动平均；未到继续前进，到位或轻微越线才停车。"""
    def __init__(self):
        self.reset()

    def reset(self, allow_start_marker=True):
        self.marker = self.reference_marker = None
        self.confirmations = self.kind_confirmations = self.settled_frames = 0
        self.stopped = self.position_confirmed = False
        self.arrived = False
        self.last_seen_at = None
        self.latched_kind = None
        self.samples = []
        self.error = self.raw_error = self.height = None
        self.ignored = None
        self.ignore_missing = 0
        self.startup_checked = not allow_start_marker
        self.startup_commanded_distance = 0.
        self.startup_ignore_unseen_travel = 0.
        self.startup_ignored = False
        self.station_id = 0
        self.ledger = StationLedger()
        self.current_identity = self.ignored_identity = None
        self.ignore_commanded_distance = 0.
        self.ignore_measured_distance = self.ignore_measured_unseen = 0.
        self.ignore_odometry_available = False
        self.station_counted = False
        self.missing = False
        self.dropout_commanded_distance = 0.
        self.settling_pending = False
        self.settling_outside_frames = 0

    def record_motion(self, speed, dt, config):
        """记录已输出的行驶距离估计；零速等待不消耗离站窗口，不代替里程计。"""
        distance = abs(speed)*max(0., dt)
        if not math.isfinite(distance):
            self.startup_checked = True
            return
        if not self.startup_checked:
            self.startup_commanded_distance += distance
            if self.startup_commanded_distance >= config.reverse_start_marker_max_travel:
                self.startup_checked = True
        if self.ignored is not None:
            self.startup_ignore_unseen_travel += distance
            self.ignore_commanded_distance += distance
        if self.missing:
            self.dropout_commanded_distance += distance

    def departure_distance(self, unseen=False):
        if self.ignore_odometry_available:
            return self.ignore_measured_unseen if unseen else self.ignore_measured_distance
        return self.startup_ignore_unseen_travel if unseen else self.ignore_commanded_distance

    def record_odometry_distance(self, distance):
        """仅输入已通过连续性检查的实际里程计增量；用于离站去重行程。"""
        if self.ignored is not None and math.isfinite(distance) and distance >= 0.:
            self.ignore_odometry_available = True
            self.ignore_measured_distance += distance
            self.ignore_measured_unseen += distance

    def observe(self, candidates, height, config, now=None):
        """每张新图像调用一次；缺失帧不以零误差混入平均值。"""
        now = monotonic() if now is None else now
        self.height = height
        forward = travel_image_sign(config)
        def distance(marker):
            return forward*(marker['y']/height-config.stop_marker_target_ratio)
        remaining = list(candidates)
        if not self.startup_checked:
            if (config.drive_direction == 'reverse' and config.reverse_ignore_start_marker
                    or config.drive_direction == 'forward' and config.forward_ignore_start_marker):
                # 前进启动时，整幅画面内最近的一根横线都可以作为出发线。
                # 只取消这一根的停车判断，不放宽后续站点的到位容差。
                initial = [m for m in remaining
                           if 0 <= m['y'] < height
                           and (config.drive_direction == 'forward'
                                or distance(m) <= config.stop_marker_position_tolerance_ratio)]
                if initial:
                    self.ignored = dict(min(initial, key=lambda m: abs(distance(m))))
                    self.ignored_identity = self.ledger.encounter(self.ignored, 'ignored_departure')
                    self.ignore_commanded_distance = 0.
                    self.startup_ignored = True
                    self.startup_ignore_unseen_travel = 0.
                    self.startup_checked = True
            if remaining:
                self.startup_checked = True
        if self.ignored is not None:
            # 离站线位于目标后侧时允许图像抖动；不能漏几帧就变成一根新横线。
            candidates = [m for m in remaining if abs(m['y']-self.ignored['y']) <= .08*height
                          or distance(m) <= config.stop_marker_position_tolerance_ratio]
            # 已经选中的下一站不能在靠近时被旧离站线吞掉。
            if self.marker is not None:
                candidates = [m for m in candidates if abs(m['y']-self.marker['y']) > .08*height]
            match = min(candidates, key=lambda m: abs(m['y']-self.ignored['y']), default=None)
            if match is not None:
                self.ignored = dict(match)
                self.ignore_missing = 0
                self.startup_ignore_unseen_travel = 0.
                self.ignore_measured_unseen = 0.
                remaining.remove(match)
                release = max(config.stop_marker_release_ratio,
                              2*config.stop_marker_overshoot_tolerance_ratio+.02)
                if distance(match) < -release and self.departure_distance() >= .005:
                    self.ledger.record('passed', self.ignored_identity, match, basis='image_past_reference')
                # 全画面离站去重：确认接近运动后侧画面边缘，才解除可见旧线的身份。
                rear_edge = match['y'] < .02*height if forward > 0 else match['y'] > .98*height
                if rear_edge and distance(match) < -release:
                    self.ignored = None
                    self.ignored_identity = None
                    self.startup_ignored = False
            else:
                self.ignore_missing += 1
                # 10mm只用于启动时是否允许首次忽略，不用于出发线漏检后的去重寿命。
                if self.departure_distance(unseen=True) >= config.stop_marker_departure_hold_distance:
                    self.ledger.record('released_unseen', self.ignored_identity)
                    self.ignored = None
                    self.ignored_identity = None
                    self.startup_ignored = False
        match = None
        same_marker = False
        if self.marker is not None:
            match = min(remaining, key=lambda m: abs(m['y']-self.marker['y']), default=None)
            same_marker = match is not None and abs(match['y']-self.marker['y']) <= .08*height
            if not same_marker:
                match = None
        if match is None and self.marker is None and not self.stopped:
            # 已选中的停车目标漏检/跳变时保留目标，不改追后面一条横线。
            # 启动时已略过中心的横线也需识别；轻微越线可作为到达。
            ahead = [m for m in remaining if distance(m) >= -2*config.stop_marker_overshoot_tolerance_ratio]
            match = min(ahead, key=lambda m: abs(distance(m)), default=None)
            if match is not None:
                self.current_identity = self.ledger.encounter(match)
        self.samples = [(t, value) for t, value in self.samples
                        if 0 <= now-t <= config.stop_marker_average_max_age_sec]
        if match is not None:
            self.last_seen_at = now
            self.dropout_commanded_distance = 0.
        if self.arrived:
            # 已确认停车是任务状态，不因后续漏检、样本过期或像素抖动重新寻线。
            # 仍更新当前观测，resume必须等真实横线重新可见，不能凭历史位置解锁。
            self.missing = match is None
            if match is not None:
                previous_kind = self.marker['kind']
                self.kind_confirmations = self.kind_confirmations+1 if match['kind'] == previous_kind else 1
                self.marker = dict(match)
                self.raw_error = distance(match)
                self.samples.append((now, self.raw_error))
                self.samples = self.samples[-config.stop_marker_average_frames:]
                self.error = fmean(value for _, value in self.samples)
                if self.latched_kind == 'unknown' and self.kind_confirmations >= config.stop_marker_confirmation_frames:
                    self.latched_kind = match['kind']
            return
        if len(self.samples) < config.stop_marker_average_frames:
            # 旧观察过期后必须重新确认，不能累加不同时段的到位证据。
            self.settled_frames = 0
            self.position_confirmed = False
        if match is None:
            # 漏检不补造位置；仅远离到位区时允许短时缓行，保留有效期内的到位计数，
            # 下次实际检测仍在容差内时继续确认，避免有效帧与漏检帧交替时永远无法到站。
            self.position_confirmed = False
            if self.marker is not None:
                self.missing = True
            return
        if not same_marker:
            self.samples.clear()
            self.confirmations = self.kind_confirmations = 0
            self.latched_kind = None
            self.reference_marker = None
            self.station_counted = False
        previous_kind = self.marker['kind'] if same_marker else None
        self.confirmations += 1
        self.kind_confirmations = self.kind_confirmations+1 if match['kind'] == previous_kind else 1
        self.marker = dict(match)
        self.missing = False
        self.raw_error = distance(match)
        if -config.stop_marker_overshoot_tolerance_ratio <= self.raw_error <= config.stop_marker_position_tolerance_ratio:
            self.settling_pending = True
            self.settling_outside_frames = 0
        elif self.settling_pending:
            self.settling_outside_frames += 1
            if self.settling_outside_frames >= config.stop_marker_average_frames:
                self.settling_pending = False
        self.samples.append((now, self.raw_error))
        self.samples = self.samples[-config.stop_marker_average_frames:]
        self.error = fmean(value for _, value in self.samples)
        ready = len(self.samples) == config.stop_marker_average_frames
        within = (ready and -config.stop_marker_overshoot_tolerance_ratio-1e-9 <= self.error
                  <= config.stop_marker_position_tolerance_ratio+1e-9)
        self.stopped = within
        if within:
            self.settled_frames += 1
            if self.reference_marker is None:
                self.reference_marker = dict(match)
            if self.latched_kind is None or self.latched_kind == 'unknown':
                self.latched_kind = (match['kind'] if self.kind_confirmations >=
                                     config.stop_marker_confirmation_frames else 'unknown')
            self.position_confirmed = self.settled_frames >= config.stop_marker_confirmation_frames
            if self.position_confirmed and not self.station_counted:
                self.arrived = True
                self.station_id += 1
                self.station_counted = True
                if (self.ignored_identity is not None and self.ignored_identity != self.current_identity
                        and self.departure_distance() >= .005):
                    self.ledger.record('passed', self.ignored_identity, self.ignored, basis='next_station_arrived')
                self.ledger.record('arrived', self.current_identity, self.marker)
        else:
            # 平均位置不在容差内就继续调整，不能被历史停车标志锁住。
            self.settled_frames = 0
            self.position_confirmed = False
            self.reference_marker = None

    def state(self, config, now=None):
        if self.arrived:
            return 'stopped_'+(self.latched_kind or 'unknown')
        if self.missing:
            age = (monotonic() if now is None else now)-self.last_seen_at if self.last_seen_at is not None else math.inf
            if (self.confirmations >= config.stop_marker_average_frames
                    and not self.settling_pending
                    and 0. <= age <= config.stop_marker_dropout_grace_sec
                    and self.dropout_commanded_distance < config.stop_marker_dropout_max_distance
                    and min(self.error, self.raw_error) > 2*config.stop_marker_position_tolerance_ratio):
                return 'missing_approaching'
            return 'missing'
        if self.marker is None or self.error is None:
            return 'searching'
        ready = len(self.samples) == config.stop_marker_average_frames
        tolerance = config.stop_marker_position_tolerance_ratio
        if ready and self.error < -config.stop_marker_overshoot_tolerance_ratio-1e-9:
            return 'overshoot'
        if self.settling_pending and (self.error > tolerance or not ready):
            # 最新实测到停车区后保持零速确认；连续一窗新帧都在区外才撤销，避免噪声启停。
            return 'settling'
        if not ready:
            return 'averaging' if self.error < -tolerance else 'approaching'
        if self.error < -config.stop_marker_overshoot_tolerance_ratio-1e-9:
            return 'overshoot'
        if self.error > tolerance+1e-9:
            return 'approaching'
        if not self.position_confirmed:
            return 'settling'
        return 'stopped_'+(self.latched_kind or 'unknown')

    def limit(self, speed, config, now=None):
        """平均位置未到时持续靠近；到位/轻微越线为零速，不自动倒车。"""
        state = self.state(config, now)
        if state == 'searching':
            return speed, state
        if state in ('approaching', 'missing_approaching'):
            cap = config.stop_marker_creep_speed
            if state == 'approaching' and len(self.samples) == config.stop_marker_average_frames:
                cap = max(cap, config.forward_speed*min(1., self.error/config.stop_marker_slowdown_ratio))
            return math.copysign(min(abs(speed), cap), speed), state
        return 0., state

    def resume(self, config):
        if self.state(config) != 'stopped_intermediate' or self.missing:
            return False
        ignored, station_id, ledger, identity = dict(self.marker), self.station_id, self.ledger, self.current_identity
        self.reset(allow_start_marker=False)
        self.ignored, self.station_id, self.ledger = ignored, station_id, ledger
        self.ignored_identity = identity
        self.ledger.record('resumed', identity, ignored)
        return True

    def snapshot(self, config):
        average_y = None
        if self.height and self.error is not None:
            forward = travel_image_sign(config)
            average_y = self.height*(config.stop_marker_target_ratio+forward*self.error)
        return {'enabled': config.stop_marker_enabled, 'state': self.state(config),
                'arrived_latched': self.arrived, 'marker_currently_missing': self.missing,
                'dropout_commanded_distance_m': self.dropout_commanded_distance,
                'dropout_max_distance_m': config.stop_marker_dropout_max_distance,
                'start_marker_ignored': self.startup_ignored,
                'reverse_start_marker_ignored': self.startup_ignored and config.drive_direction == 'reverse',
                'startup_commanded_distance_m': self.startup_commanded_distance,
                'kind': self.latched_kind, 'station_id': self.station_id,
                'station_counts': dict(self.ledger.counts), 'station_history': self.ledger.snapshot(),
                'current_station_number': self.current_identity, 'ignored_station_number': self.ignored_identity,
                'departure_hold_distance_m': config.stop_marker_departure_hold_distance,
                'departure_measured_unseen_m': self.ignore_measured_unseen,
                'marker': self.marker, 'error_ratio': self.error, 'raw_error_ratio': self.raw_error,
                'average_y': average_y, 'sample_count': len(self.samples),
                'average_frames': config.stop_marker_average_frames, 'filter': 'arithmetic_mean',
                'position_tolerance_ratio': config.stop_marker_position_tolerance_ratio,
                'position_confirmed': self.position_confirmed,
                'settling_on_fresh_position': self.settling_pending,
                'overshoot_tolerance_ratio': config.stop_marker_overshoot_tolerance_ratio,
                'overshoot': self.state(config) == 'overshoot',
                'target_ratio': config.stop_marker_target_ratio,
                'waiting_for_resume': config.stop_marker_enabled and self.state(config) == 'stopped_intermediate'}
