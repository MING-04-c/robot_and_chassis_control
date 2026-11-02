"""前置相机倒车：局部主线估计、可选地面映射及全向底盘视觉伺服。"""
import json
import math
from pathlib import Path
from statistics import median

import cv2
import numpy as np


class GroundMapping:
    """读取实测对应点JSON；输入处理图像像素，输出base_link地面X前/Y左，单位米。"""
    def __init__(self, filename):
        path = Path(filename).expanduser()
        if path.stat().st_size > 65536:
            raise ValueError('ground calibration file too large')
        data = json.loads(path.read_text())
        if data.get('frame_id') != 'base_link' or data.get('units') != 'm':
            raise ValueError('ground mapping requires frame_id=base_link, units=m')
        self.size = tuple(data['image_size'])
        if len(self.size) != 2 or any(type(v) is not int or v <= 1 for v in self.size):
            raise ValueError('invalid ground mapping image_size')
        pixels = np.asarray(data['image_points_px'], dtype=float)
        ground = np.asarray(data['ground_points_m'], dtype=float)
        if (pixels.shape != ground.shape or pixels.ndim != 2 or pixels.shape[1] != 2
                or not 4 <= len(pixels) <= 64 or not np.isfinite(pixels).all()
                or not np.isfinite(ground).all()):
            raise ValueError('ground mapping requires >=4 finite corresponding points')
        if np.any(pixels < 0) or np.any(pixels >= np.asarray(self.size)):
            raise ValueError('calibration pixels outside image')
        self.h, _ = cv2.findHomography(pixels, ground, 0)
        if self.h is None or not np.isfinite(self.h).all() or np.linalg.matrix_rank(self.h) < 3:
            raise ValueError('degenerate ground mapping')
        mapped = self.project(pixels)
        self.fit_error_m = float(np.max(np.linalg.norm(mapped-ground, axis=1)))
        if self.fit_error_m > .01:
            raise ValueError('ground calibration fit exceeds 1cm; check measured points')

    def project(self, points):
        """输入Nx2像素，输出Nx2地面米制坐标；拒绝地平线及非有限投影。"""
        values = np.column_stack((points, np.ones(len(points)))) @ self.h.T
        if np.any(np.abs(values[:, 2]) < 1e-8):
            raise ValueError('ground mapping horizon')
        result = values[:, :2]/values[:, 2, None]
        if not np.isfinite(result).all() or np.any(np.abs(result) > 10):
            raise ValueError('ground projection out of local range')
        return result


def robust_axis(points, residual_limit):
    """拟合x=a*y+b，输出(a,b,内点)；中位数初估抑制横线产生的错误中线点。"""
    if len(points) < 6:
        return None
    points = np.asarray(points, dtype=float)
    slopes = [(a[0]-b[0])/(a[1]-b[1]) for i, a in enumerate(points)
              for b in points[i+1:] if abs(a[1]-b[1]) > 1e-5]
    if not slopes:
        return None
    slope = median(slopes)
    intercept = float(np.median(points[:, 0]-slope*points[:, 1]))
    good = points[np.abs(points[:, 0]-slope*points[:, 1]-intercept) <= residual_limit]
    if len(good) < max(6, math.ceil(.65*len(points))):
        return None
    if float(np.ptp(good[:, 1])) < 1e-5:
        return None
    slope, intercept = np.polyfit(good[:, 1], good[:, 0], 1)
    return float(slope), float(intercept), good


def near_line(points, mask, config, mapping=None):
    """同帧中线+二值掩膜→近车头局部轴；剔除横线宽行，不使用远端弯道。"""
    height, width = mask.shape
    forward = 1. if config.image_forward_direction == 'down' else -1.
    reference = max(0., min(height-1., config.vehicle_reference_y_ratio*(height-1)))
    start = max(0., min(height-1., reference+forward*config.near_edge_margin_ratio*(height-1)))
    candidates = sorted(((float(x), float(y)) for x, y in points
                         if 0 <= forward*(y-start) <= .24*(height-1)
                         and 0 <= x < width and 0 <= y < height), key=lambda p: forward*p[1])
    rows = []
    for x, y in candidates[::max(1, len(candidates)//32)]:
        row = mask[int(round(y))]
        ix = min(width-1, int(round(x)))
        if not row[ix]:
            continue
        left = np.flatnonzero(row[:ix] == 0)
        right = np.flatnonzero(row[ix:] == 0)
        run = (ix+int(right[0]) if len(right) else width)-(int(left[-1])+1 if len(left) else 0)
        rows.append((x, y, run))
    if len(rows) < 6:
        return None
    normal_width = median(p[2] for p in rows)
    clean = [(x, y) for x, y, run in rows if run <= max(.07*width, 2.5*normal_width)]
    axis = robust_axis(clean, .0125*width)
    if axis is None:
        return None
    slope, intercept, good = axis
    if float(np.ptp(good[:, 1])) < .08*(height-1):
        return None
    near_y = float(good[np.argmin(forward*(good[:, 1]-start)), 1])
    error = (slope*near_y+intercept-config.vehicle_reference_x_ratio*(width-1))/((width-1)/2)
    feedback = dict(error_normalized=float(error), observed_error_normalized=float(error),
                    near_heading_rad=math.atan(forward*slope), source='near_mask_axis',
                    axis_pixels=[[float(slope*y+intercept), float(y)]
                                 for y in (float(good[:, 1].min()), float(good[:, 1].max()))],
                    span_px=float(np.ptp(good[:, 1])), removed_wide_rows=len(rows)-len(clean),
                    metric=False)
    if mapping is not None:
        if mapping.size != (width, height):
            raise ValueError('ground mapping resolution mismatch; recalibration required')
        ground = mapping.project(good)
        # 地面线Y=a*X+b，车身中心处的b才是横向偏差，不是图像中心像素。
        axis = robust_axis(ground[:, ::-1], .01)
        if axis is None or float(np.ptp(ground[:, 0])) < .04:
            return None
        a, b, _ = axis
        feedback.update(lateral_error_m=b, near_heading_rad=math.atan(a),
                        source='ground_near_axis', metric=True)
    return feedback


def aligned(feedback, route):
    if feedback is None:
        return False
    lateral = (abs(feedback['lateral_error_m']) <= route.visual_goal_lateral_m
               if feedback.get('metric') else abs(feedback['error_normalized']) <= route.finish_goal_lateral)
    return lateral and abs(feedback['near_heading_rad']) <= route.finish_goal_heading


def strafe_command(feedback, config, route):
    """输出(vy,w)，m/s、rad/s；平移与朝向独立，不随倒车反转横移方向。"""
    metric = feedback.get('metric', False)
    error = feedback['lateral_error_m'] if metric else feedback['error_normalized']
    tolerance = route.visual_goal_lateral_m if metric else route.finish_goal_lateral
    gain = route.visual_lateral_gain if metric else route.visual_image_lateral_gain
    sign = 1. if metric else (config.steering_sign if config.image_forward_direction == 'up' else -config.steering_sign)
    # 分阶段控制：进入到位区就停本轴，不继续追微小误差；有效命令高于底盘锁车阈值。
    side = math.copysign(max(0., abs(error)-tolerance), error)
    heading = feedback['near_heading_rad']
    angle = math.copysign(max(0., abs(heading)-route.finish_goal_heading), heading)
    vy = sign*gain*side
    w = sign*route.finish_heading_gain*angle
    if side:
        vy = math.copysign(max(abs(vy), route.visual_min_lateral_speed), vy)
    if angle:
        w = math.copysign(max(abs(w), route.visual_min_angular_speed), w)
    return (max(-route.visual_lateral_speed, min(route.visual_lateral_speed, vy)),
            max(-.06, min(.06, w)))


class AlignmentSequencer:
    """单轴分阶段对齐；新帧确认、行走迟滞及停稳间隔，避免舵轮频繁换方向。"""
    def __init__(self):
        self.phase = None
        self.pending = None
        self.frames = 0
        self.last_stamp = None
        self.settle_until = 0.
        self.axis_direction = self.direction_candidate = self.direction_frames = 0

    def update(self, feedback, route, stamp, now, terminal=False, stopped=True):
        new_frame = stamp != self.last_stamp
        error = abs(feedback['lateral_error_m'] if feedback.get('metric') else feedback['error_normalized'])
        tolerance = route.visual_goal_lateral_m if feedback.get('metric') else route.finish_goal_lateral
        heading = abs(feedback['near_heading_rad'])
        # 倒车恢复后，小噪声不立即让轮子转90度回去横移；最终到位仍用原始严格容差。
        release = 1.5 if self.phase == 'ready' and not terminal else 1.
        target = ('heading' if heading > release*route.finish_goal_heading else
                  'lateral' if error > release*tolerance else 'ready')
        if target != self.phase:
            if stamp != self.last_stamp:
                self.frames = self.frames+1 if self.pending == target else 1
                self.pending = target
            if self.frames >= 3:
                self.phase, self.pending, self.frames = target, None, 0
                self.settle_until = now+.35
                self.axis_direction = self.direction_candidate = self.direction_frames = 0
            self.last_stamp = stamp
            return 'settling'
        self.last_stamp = stamp
        self.pending, self.frames = None, 0
        if now < self.settle_until or not stopped and self.settle_until:
            return 'settling'
        self.settle_until = 0.
        if self.phase in ('heading', 'lateral'):
            value = (feedback['near_heading_rad'] if self.phase == 'heading' else
                     feedback['lateral_error_m'] if feedback.get('metric') else feedback['error_normalized'])
            direction = 1 if value >= 0 else -1
            if self.axis_direction and direction != self.axis_direction:
                if new_frame:
                    self.direction_frames = self.direction_frames+1 if self.direction_candidate == direction else 1
                    self.direction_candidate = direction
                if self.direction_frames >= 3:
                    self.axis_direction = direction
                    self.direction_frames = 0
                    self.settle_until = now+.35
                return 'settling'
            self.axis_direction = direction
            self.direction_candidate = self.direction_frames = 0
        return self.phase
