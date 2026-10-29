"""垂直于主线的停止横线识别，以及居中停车/等待新消息继续的状态机。"""
import math
from statistics import median

import cv2
import numpy as np


def detect_stop_markers(mask, points, config):
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
                      'span': length, 'angle_rad': math.atan2(dy, dx)})
    return found


class StopMarkerGate:
    """只根据新图像累计确认；停车锁存，途中站收到新的 resume 消息才解除。"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.marker = None
        self.confirmations = self.kind_confirmations = 0
        self.stopped = False
        self.latched_kind = None
        self.settled_frames = 0
        self.ignored = None
        self.ignore_missing = 0
        self.station_id = 0
        self.height = None
        self.error = None
        self.missing = False

    def observe(self, candidates, height, config):
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
        if self.marker is not None:
            match = min(remaining, key=lambda m: abs(m['y']-self.marker['y']), default=None)
            if match is not None and abs(match['y']-self.marker['y']) > .08*height:
                match = None
        if match is None and not self.stopped:
            ahead = [m for m in remaining if distance(m) >= -config.stop_marker_position_tolerance_ratio]
            match = min(ahead, key=distance, default=None)
        if match is None:
            if not self.stopped:
                if (self.confirmations >= config.stop_marker_confirmation_frames
                        and self.error is not None and self.error <= config.stop_marker_slowdown_ratio):
                    self.missing = True
                    return
                self.marker = None
                self.confirmations = self.kind_confirmations = 0
            return
        if self.marker is not None and abs(match['y']-self.marker['y']) <= .08*height:
            self.confirmations += 1
            self.kind_confirmations = self.kind_confirmations+1 if match['kind'] == self.marker['kind'] else 1
        else:
            self.confirmations = self.kind_confirmations = 1
        self.marker = dict(match)
        self.missing = False
        self.error = distance(match)
        if self.stopped:
            self.settled_frames = (self.settled_frames+1 if abs(self.error) <=
                                   config.stop_marker_position_tolerance_ratio else 0)
        if (self.stopped and self.latched_kind == 'unknown'
                and self.kind_confirmations >= config.stop_marker_confirmation_frames
                and match['kind'] != 'unknown'):
            self.latched_kind = match['kind']

    def state(self, config):
        if self.missing and not self.stopped:
            return 'missing'
        if not self.stopped:
            return 'approaching' if self.marker else 'searching'
        if self.error is not None and self.error < -config.stop_marker_position_tolerance_ratio:
            return 'overshoot'
        if self.settled_frames < config.stop_marker_confirmation_frames:
            return 'settling'
        return 'stopped_'+(self.latched_kind or 'unknown')

    def limit(self, speed, config):
        """返回限速与站点状态；停车指令绕过缓降，实际位置误差仍需图像复核。"""
        state = self.state(config)
        if self.stopped or self.missing:
            return 0.0, state
        if self.marker is None or self.error is None:
            return speed, state
        if self.error <= config.stop_marker_position_tolerance_ratio:
            if self.confirmations >= config.stop_marker_confirmation_frames:
                self.stopped = True
                self.settled_frames = 0
                self.latched_kind = (self.marker['kind'] if self.kind_confirmations >=
                                     config.stop_marker_confirmation_frames else 'unknown')
                self.station_id += 1
                return 0.0, self.state(config)
            return 0.0, 'confirming'
        cap = max(config.stop_marker_creep_speed,
                  config.forward_speed*min(1.0, self.error/config.stop_marker_slowdown_ratio))
        return min(speed, cap), 'approaching'

    def resume(self, config):
        if self.state(config) != 'stopped_intermediate':
            return False
        self.ignored = dict(self.marker)
        self.ignore_missing = 0
        self.missing = False
        self.marker = None
        self.confirmations = self.kind_confirmations = 0
        self.stopped = False
        self.latched_kind = None
        self.error = None
        return True

    def snapshot(self, config):
        return {'enabled': config.stop_marker_enabled, 'state': self.state(config),
                'kind': self.latched_kind, 'station_id': self.station_id,
                'marker': self.marker, 'error_ratio': self.error,
                'overshoot': self.error is not None and self.error < -config.stop_marker_position_tolerance_ratio,
                'target_ratio': config.stop_marker_target_ratio,
                'waiting_for_resume': config.stop_marker_enabled and self.state(config) == 'stopped_intermediate'}
