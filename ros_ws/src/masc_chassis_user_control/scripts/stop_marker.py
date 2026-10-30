"""垂直于主线的停止横线识别，以及居中停车/等待新消息继续的状态机。"""
import math
from statistics import median, fmean
from time import monotonic

import cv2
import numpy as np


def detect_stop_markers(mask, points, config, reference=None):
    """输入同尺寸单通道蓝线掩膜、实测中线点；输出横线交点及后续主线类别。"""
    if mask is None or mask.ndim != 2 or len(points) < 3:
        return []
    height, width = mask.shape
    if any(not math.isfinite(v) for point in points for v in point):
        return []
    points = sorted(((float(x), float(y)) for x, y in points), key=lambda p: p[1])
    path_x, path_y = [p[0] for p in points], [p[1] for p in points]
    forward = 1.0 if config.image_forward_direction == 'down' else -1.0
    binary = np.uint8(mask > 0) * 255
    edges = cv2.Canny(binary, 40, 100)
    lines = cv2.HoughLinesP(edges, 1, math.pi / 180, 20,
                           minLineLength=max(20, int(width * config.stop_marker_min_span_ratio)),
                           maxLineGap=max(5, int(width * 0.05)))
    if lines is None:
        return []
    found = []
    proposals = sorted(lines[:, 0], key=lambda p: math.hypot(p[2]-p[0], p[3]-p[1]), reverse=True)[:40]
    for x1, y1, x2, y2 in proposals:
        dx, dy = float(x2-x1), float(y2-y1)
        length = math.hypot(dx, dy)
        middle_y = (y1+y2) / 2.0
        local = [(x, y) for x, y in points if abs(y-middle_y) <= 0.14 * height]
        slopes = [(a[0]-b[0])/(a[1]-b[1]) for i, a in enumerate(local)
                  for b in local[i+1:] if abs(a[1]-b[1]) >= 3]
        if not slopes or len(local) < 3 or max(y for _, y in local)-min(y for _, y in local) < .04*height:
            continue
        slope = median(slopes)
        intercept = median(x-slope*y for x, y in local)
        if (reference is not None and 'main_slope' in reference
                and abs(middle_y-reference['y']) < .08*height):
            # 停车后相机朝向不变，固定主线参考轴；横线位置仍每帧从真实像素重测。
            slope = reference['main_slope']
            intercept = reference['x']-slope*reference['y']
        if abs(slope*dx+dy) / (math.hypot(slope, 1)*length) > math.sin(config.stop_marker_angle_tolerance_rad):
            continue
        denominator = dx-slope*dy
        if abs(denominator) < 1e-6:
            continue
        fraction = (slope*y1+intercept-x1)/denominator
        arm = width*config.stop_marker_min_arm_ratio
        if min(fraction, 1-fraction)*length < arm:
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
            continue
        offsets = normal[wing]
        if np.percentile(offsets, 95)-np.percentile(offsets, 5) > .06*height:
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
    return found


class StopMarkerGate:
    """对同一横线的新帧位置滚动平均；未到继续前进，到位或轻微越线才停车。"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.marker = self.reference_marker = None
        self.confirmations = self.kind_confirmations = self.settled_frames = 0
        self.stopped = self.position_confirmed = False
        self.latched_kind = None
        self.samples = []
        self.error = self.raw_error = self.height = None
        self.ignored = None
        self.ignore_missing = 0
        self.station_id = 0
        self.station_counted = False
        self.missing = False

    def observe(self, candidates, height, config, now=None):
        """每张新图像调用一次；缺失帧不以零误差混入平均值。"""
        now = monotonic() if now is None else now
        self.height = height
        forward = 1.0 if config.image_forward_direction == 'down' else -1.0
        def distance(marker):
            return forward*(marker['y']/height-config.stop_marker_target_ratio)
        remaining = list(candidates)
        if self.ignored is not None:
            match = min(remaining, key=lambda m: abs(m['y']-self.ignored['y']), default=None)
            if match is not None and abs(match['y']-self.ignored['y']) <= .08*height:
                self.ignored = dict(match)
                self.ignore_missing = 0
                remaining.remove(match)
                if distance(match) < -config.stop_marker_release_ratio:
                    self.ignored = None
            else:
                self.ignore_missing += 1
                if self.ignore_missing >= config.stop_marker_confirmation_frames:
                    self.ignored = None
        match = None
        same_marker = False
        if self.marker is not None:
            match = min(remaining, key=lambda m: abs(m['y']-self.marker['y']), default=None)
            same_marker = match is not None and abs(match['y']-self.marker['y']) <= .08*height
            if not same_marker:
                match = None
        if match is None and not self.stopped:
            # 启动时已略过中心的横线也需识别；轻微越线可作为到达。
            ahead = [m for m in remaining if distance(m) >= -2*config.stop_marker_overshoot_tolerance_ratio]
            match = min(ahead, key=lambda m: abs(distance(m)), default=None)
        self.samples = [(t, value) for t, value in self.samples
                        if 0 <= now-t <= config.stop_marker_average_max_age_sec]
        if match is None:
            self.settled_frames = 0
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
                self.station_id += 1
                self.station_counted = True
        else:
            # 平均位置不在容差内就继续调整，不能被历史停车标志锁住。
            self.settled_frames = 0
            self.position_confirmed = False
            self.reference_marker = None

    def state(self, config):
        if self.missing:
            return 'missing'
        if self.marker is None or self.error is None:
            return 'searching'
        ready = len(self.samples) == config.stop_marker_average_frames
        tolerance = config.stop_marker_position_tolerance_ratio
        if not ready:
            return 'averaging' if self.error < -tolerance else 'approaching'
        if self.error < -config.stop_marker_overshoot_tolerance_ratio-1e-9:
            return 'overshoot'
        if self.error > tolerance+1e-9:
            return 'approaching'
        if not self.position_confirmed:
            return 'settling'
        return 'stopped_'+(self.latched_kind or 'unknown')

    def limit(self, speed, config):
        """平均位置未到时持续靠近；到位/轻微越线为零速，不自动倒车。"""
        state = self.state(config)
        if state == 'searching':
            return speed, state
        if state == 'approaching':
            cap = config.stop_marker_creep_speed
            if len(self.samples) == config.stop_marker_average_frames:
                cap = max(cap, config.forward_speed*min(1., self.error/config.stop_marker_slowdown_ratio))
            return min(max(0., speed), cap), state
        return 0., state

    def resume(self, config):
        if self.state(config) != 'stopped_intermediate':
            return False
        ignored, station_id = dict(self.marker), self.station_id
        self.reset()
        self.ignored, self.station_id = ignored, station_id
        return True

    def snapshot(self, config):
        average_y = None
        if self.height and self.error is not None:
            forward = 1. if config.image_forward_direction == 'down' else -1.
            average_y = self.height*(config.stop_marker_target_ratio+forward*self.error)
        return {'enabled': config.stop_marker_enabled, 'state': self.state(config),
                'kind': self.latched_kind, 'station_id': self.station_id,
                'marker': self.marker, 'error_ratio': self.error, 'raw_error_ratio': self.raw_error,
                'average_y': average_y, 'sample_count': len(self.samples),
                'average_frames': config.stop_marker_average_frames, 'filter': 'arithmetic_mean',
                'position_tolerance_ratio': config.stop_marker_position_tolerance_ratio,
                'position_confirmed': self.position_confirmed,
                'overshoot_tolerance_ratio': config.stop_marker_overshoot_tolerance_ratio,
                'overshoot': self.state(config) == 'overshoot',
                'target_ratio': config.stop_marker_target_ratio,
                'waiting_for_resume': config.stop_marker_enabled and self.state(config) == 'stopped_intermediate'}
