#!/usr/bin/env python3
"""Unit checks for direction, lost-line stop, slowdown and parameter validation."""

import math
import sys
import unittest

import cv2
import numpy as np
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from stop_marker import StopMarkerGate, detect_stop_markers
from line_follow_control import ControlConfig, line_command, motion_preview, tracking_geometry, slew, smooth_yaw  # noqa: E402


raw_tracking_geometry = tracking_geometry


def tracking_geometry(points, width, height, config):
    """原几何用例以稀疏控制点描述连续胶带，补成实际逐行采样；断线用例使用 raw。"""
    dense = []
    for first, second in zip(points, points[1:]):
        steps = max(1, math.ceil(abs(second[1] - first[1]) / 6)) if all(
            math.isfinite(v) for v in (*first, *second)) else 1
        dense.extend((first[0] + (second[0] - first[0]) * i / steps,
                      first[1] + (second[1] - first[1]) * i / steps) for i in range(steps))
    if points:
        dense.append(points[-1])
    return raw_tracking_geometry(dense, width, height, config)


def observation(**updates):
    value = {
        'lost': False, 'processing_over_budget': False, 'orientation': 'vertical',
        'image_width': 640, 'image_height': 480,
        'bottom_connected': True, 'error_normalized': 0.0,
        'valid_row_fraction': 0.8, 'target_estimated': False,
        'target_fallback': False, 'occlusion_suspected': False,
    }
    value.update(updates)
    return value


class ControlTests(unittest.TestCase):
    def test_lookahead_does_not_jump_with_scan_row_phase(self):
        config = ControlConfig()
        targets = []
        for phase in range(6):
            points = [(320+.3*(y-200), y) for y in range(120+phase, 470, 6)]
            geometry, reason = raw_tracking_geometry(points, 640, 480, config)
            self.assertIsNone(reason)
            targets.append((geometry['target_x'], geometry['target_y']))
            self.assertAlmostEqual(geometry['target_y'], .4*479)
            self.assertFalse(geometry['target_gap_interpolated'])
        for target in targets[1:]:
            self.assertAlmostEqual(target[0], targets[0][0])

    def test_bend_continuity_margin_keeps_supported_turn_and_rejects_large_gap(self):
        config = ControlConfig()
        near = [(300 + .4*(y-260), y) for y in range(150, 222, 6)]
        far = [(300 - .4*(y-260), y) for y in range(300, 450, 6)]
        # 约44度转角，两端切线在缺口内相交，不应截断成近端短线。
        joined, reason = raw_tracking_geometry(near+far, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(len(joined['bridged_gaps']), 1)
        self.assertGreater(joined['trusted_end_y'], 430)
        old, _ = raw_tracking_geometry(near+far, 640, 480,
                                      replace(config, bridge_intersection_angle_rad=.60))
        self.assertEqual(old['bridged_gaps'], [])
        # 更晚出现的近端弯线可接受；完全丢失和过远的线仍拒绝。
        visible = [(430-.4*(y-310), y) for y in range(310, 470, 6)]
        self.assertIsNone(raw_tracking_geometry(visible, 640, 480, config)[1])
        self.assertEqual(raw_tracking_geometry(visible, 640, 480,
                         replace(config, max_near_gap_ratio=.35))[1], 'near_line_missing')
        self.assertEqual(raw_tracking_geometry(visible[5:], 640, 480, config)[1],
                         'near_line_missing')
        self.assertIsNone(raw_tracking_geometry([], 640, 480, config)[0])

    def test_noisy_gap_endpoint_does_not_override_supported_direction(self):
        config = ControlConfig()
        near = [(320, y) for y in range(120, 220, 6)]
        far = [(320, y) for y in range(320, 470, 6)]
        near[-1] = (340, near[-1][1])
        far[0] = (300, far[0][1])
        geometry, reason = raw_tracking_geometry(near + far, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(len(geometry['bridged_gaps']), 1)
        self.assertLessEqual(geometry['trusted_speed_factor'], config.bridge_speed_factor)

    def test_intersecting_extensions_join_only_inside_gap(self):
        config = ControlConfig()
        near = [(320 + 0.28 * (y - 216), y) for y in range(120, 220, 6)]
        far = [(320 - 0.28 * (y - 320), y) for y in range(320, 470, 6)]
        result, reason = raw_tracking_geometry(near + far, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(len(result['bridged_gaps']), 1)
        self.assertLessEqual(result['trusted_speed_factor'], config.bridge_speed_factor)
        # 平行错位以及仅在缺口外相交，都不能因为“无限延长会相交”而接入。
        outside = [(390 - 0.28 * (y - 320), y) for y in range(320, 470, 6)]
        rejected, reason = raw_tracking_geometry(near + outside, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(rejected['bridged_gaps'], [])
        mirrored, reason = raw_tracking_geometry([(x, 479-y) for x, y in near+far],
            640, 480, replace(config, image_forward_direction='up', vehicle_reference_y_ratio=1.05))
        self.assertIsNone(reason)
        self.assertEqual(len(mirrored['bridged_gaps']), 1)

    def test_short_gap_requires_both_aligned_ends_and_joins(self):
        config = ControlConfig()
        near = [(320, y) for y in range(120, 220, 6)]
        far = [(320, y) for y in range(260, 470, 6)]
        geometry, reason = raw_tracking_geometry(near + far, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(len(geometry['bridged_gaps']), 1)
        self.assertGreater(geometry['trusted_end_y'], 450)
        self.assertLessEqual(geometry['trusted_speed_factor'], config.bridge_speed_factor)
        # 反光区更长时，两侧都有足够长的实测线且几何吻合，仍允许低速通过。
        long_gap, reason = raw_tracking_geometry(
            near + [(320, y) for y in range(320, 470, 6)], 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(len(long_gap['bridged_gaps']), 1)
        self.assertGreater(long_gap['trusted_end_y'], 450)
        # 两端缺少支撑、错位、方向冲突或缺口过长都不能放行。
        cases = [near + [(380, y) for _, y in far],
                 near + [(320 + (y - 260) * 1.2, y) for _, y in far],
                 near + [(320, y) for y in range(380, 470, 6)],
                 near + [(320, y) for y in range(260, 290, 6)],
                 near + [(320, 260)]]
        for points in cases:
            result, reason = raw_tracking_geometry(points, 640, 480, config)
            self.assertIsNone(reason)
            self.assertEqual(result['bridged_gaps'], [])
            self.assertEqual(result['trusted_end_y'], near[-1][1])
        disabled, _ = raw_tracking_geometry(near + far, 640, 480,
                                           replace(config, max_bridge_gap_ratio=0.0))
        self.assertEqual(disabled['bridged_gaps'], [])
        up, reason = raw_tracking_geometry([(x, 479 - y) for x, y in near + far],
            640, 480, replace(config, image_forward_direction='up', vehicle_reference_y_ratio=1.05))
        self.assertIsNone(reason)
        self.assertEqual(len(up['bridged_gaps']), 1)

    def test_confirmed_gap_matches_continuous_line_speed_and_interpolates_goal(self):
        config = ControlConfig(lookahead_distance_ratio=0.6)
        near = [(319.5, y) for y in range(120, 220, 6)]
        far = [(319.5, y) for y in range(320, 470, 6)]
        joined, reason = raw_tracking_geometry(near + far, 640, 480, config)
        self.assertIsNone(reason)
        self.assertTrue(joined['target_gap_interpolated'])
        self.assertFalse(joined['target_fallback'])
        self.assertAlmostEqual(joined['target_y'], (-0.05 + 0.6) * 479)
        self.assertAlmostEqual(joined['target_x'], 319.5)
        self.assertEqual(joined['trusted_speed_factor'], 1.0)
        continuous, _ = raw_tracking_geometry(
            [(319.5, y) for y in range(120, 470, 6)], 640, 480, config)
        speeds = []
        for geometry in (joined, continuous):
            status = observation()
            status.update({key: geometry[key] for key in (
                'error_normalized', 'heading_error_rad', 'target_fallback', 'trusted_speed_factor')})
            speed, _, state = line_command(status, config)
            self.assertEqual(state, 'tracking')
            speeds.append(speed)
        self.assertEqual(speeds, [config.forward_speed, config.forward_speed])
        # 不吻合的远段仍截断；不能为了目标落在前方而无条件插值。
        rejected, _ = raw_tracking_geometry(
            near + [(400, y) for _, y in far], 640, 480, config)
        self.assertFalse(rejected['bridged_gaps'])
        self.assertFalse(rejected['target_gap_interpolated'])
        self.assertLessEqual(rejected['target_y'], near[-1][1])
        self.assertLess(rejected['trusted_speed_factor'], 1.0)
        limited, _ = raw_tracking_geometry(
            near + far, 640, 480, replace(config, bridge_speed_factor=0.5))
        self.assertEqual(limited['trusted_speed_factor'], 0.5)

    def test_short_near_segment_can_advance_without_a_far_segment(self):
        config = ControlConfig()
        geometry, reason = raw_tracking_geometry(
            [(320, y) for y in range(120, 156, 6)], 640, 480, config)
        self.assertIsNone(reason)
        status = observation(valid_row_fraction=geometry['trusted_span_ratio'])
        status.update({key: geometry[key] for key in (
            'error_normalized', 'heading_error_rad', 'target_fallback', 'trusted_speed_factor')})
        speed, _, reason = line_command(status, config)
        self.assertEqual(reason, 'tracking')
        self.assertGreater(speed, 0)
        self.assertLess(speed, config.forward_speed)
        self.assertEqual(raw_tracking_geometry(
            [(320, y) for y in range(120, 144, 6)], 640, 480, config)[1],
            'insufficient_forward_span')

    def test_trusted_prefix_never_jumps_gap_and_extends_with_new_frames(self):
        config = ControlConfig()
        near = [(320, y) for y in range(120, 270, 6)]
        distant = [(500, y) for y in range(350, 470, 6)]
        geometry, reason = raw_tracking_geometry(near + distant, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(geometry['trusted_end_y'], near[-1][1])
        self.assertLessEqual(geometry['target_y'], near[-1][1])
        self.assertLess(geometry['trusted_speed_factor'], 1)
        up = replace(config, image_forward_direction='up', vehicle_reference_y_ratio=1.05)
        mirrored, reason = raw_tracking_geometry(
            [(x, 479 - y) for x, y in near + distant], 640, 480, up)
        self.assertIsNone(reason)
        self.assertEqual(mirrored['trusted_end_y'], 479 - near[-1][1])
        # 新帧中缺口被真实蓝线填满，可信范围才允许延伸到远端。
        extended = [(320, y) for y in range(120, 470, 6)]
        newer, reason = raw_tracking_geometry(extended, 640, 480, config)
        self.assertIsNone(reason)
        self.assertGreater(newer['trusted_end_y'], geometry['trusted_end_y'])
        self.assertGreater(newer['trusted_speed_factor'], geometry['trusted_speed_factor'])
        # 近段不足时，即使远处还有长线，也必须停下。
        short = [(320, y) for y in range(120, 144, 6)]
        self.assertEqual(raw_tracking_geometry(short + distant, 640, 480, config)[1],
                         'insufficient_forward_span')

    def test_stop_cross_final_t_and_turn_are_distinguished(self):
        config = ControlConfig()
        for kind in ('intermediate', 'final', 'bend', 'plain'):
            with self.subTest(kind=kind):
                mask = np.zeros((480, 640), np.uint8)
                end = 240 if kind == 'final' else 460
                cv2.line(mask, (320, 20), (320, end), 255, 16)
                if kind != 'plain':
                    cv2.line(mask, (320 if kind == 'bend' else 250, 240), (390, 240), 255, 16)
                points = [(320, y) for y in range(24, end, 6)]
                markers = detect_stop_markers(mask, points, config)
                if kind in ('bend', 'plain'):
                    self.assertEqual(markers, [])
                else:
                    self.assertEqual(len(markers), 1)
                    self.assertEqual(markers[0]['kind'], kind)
                    self.assertAlmostEqual(markers[0]['y'], 240, delta=2)
                    flipped = detect_stop_markers(cv2.flip(mask, 0), [(x, 479-y) for x, y in points],
                        replace(config, image_forward_direction='up', vehicle_reference_y_ratio=1.05))
                    self.assertEqual(flipped[0]['kind'], kind)
                    self.assertAlmostEqual(flipped[0]['y'], 239, delta=2)
        # 横线在图像边缘时没有足够后方视野，不能直接当成终点。
        mask = np.zeros((480, 640), np.uint8)
        cv2.line(mask, (320, 20), (320, 445), 255, 16)
        cv2.line(mask, (250, 445), (390, 445), 255, 16)
        markers = detect_stop_markers(mask, [(320, y) for y in range(24, 445, 6)], config)
        self.assertEqual(markers[0]['kind'], 'unknown')

    def test_marker_rotation_multiple_stations_and_overshoot_hold(self):
        config = ControlConfig()
        points = [(320+.3*(y-240), y) for y in range(24, 460, 6)]
        mask = np.zeros((480, 640), np.uint8)
        cv2.polylines(mask, [np.array(points, np.int32)], False, 255, 16)
        cv2.line(mask, (250, 261), (390, 219), 255, 16)
        marker = detect_stop_markers(mask, points, config)[0]
        self.assertEqual(marker['kind'], 'intermediate')
        self.assertAlmostEqual(marker['y'], 240, delta=2)
        mask[:] = 0
        cv2.line(mask, (320, 20), (320, 320), 255, 16)
        for y in (100, 320):
            cv2.line(mask, (250, y), (390, y), 255, 16)
        markers = sorted(detect_stop_markers(mask, [(320, y) for y in range(24, 320, 6)], config),
                         key=lambda m: m['y'])
        self.assertEqual([m['kind'] for m in markers], ['intermediate', 'final'])
        gate = StopMarkerGate()
        for y in (290, 265, 240):
            for _ in range(7):
                gate.observe([{'x': 320., 'y': float(y), 'kind': 'final'}], 480, config)
            gate.limit(.075, config)
        for _ in range(7):
            gate.observe([{'x': 320., 'y': 230., 'kind': 'final'}], 480, config)
        self.assertEqual(gate.limit(.075, config), (0., 'overshoot'))
        self.assertFalse(gate.resume(config))

    def test_station_stays_stopped_until_new_resume_and_does_not_retrigger(self):
        config = ControlConfig()
        gate = StopMarkerGate()
        def observe(y, kind='intermediate', count=7):
            for _ in range(count):
                gate.observe([{'x': 320., 'y': float(y), 'kind': kind}], 480, config)
        self.assertFalse(gate.resume(config))  # 提前到达的消息不能解锁未来站点。
        observe(330)
        speed, state = gate.limit(.075, config)
        self.assertTrue(0 < speed < .075)
        self.assertEqual(state, 'approaching')
        for y in (310, 290, 270, 250, 242):
            observe(y)
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        observe(242)
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        self.assertEqual(gate.limit(.075, config)[0], 0.)
        observe(242, 'final')  # 停车后短暂丢失继续线，不能把途中站变成终点。
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        self.assertTrue(gate.resume(config))
        observe(242)
        self.assertEqual(gate.limit(.075, config)[0], .075)  # 同一根线不会立即二次停车。
        for y in (220, 200, 190):
            observe(y)
        observe(330, 'final')
        for y in (310, 290, 270, 250, 240):
            observe(y, 'final')
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_final'))
        observe(240, 'final')
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_final'))
        self.assertFalse(gate.resume(config))
        self.assertEqual(gate.station_id, 2)

    def test_station_mean_filters_jitter_and_uses_tighter_tolerance(self):
        config = ControlConfig()
        gate = StopMarkerGate()
        def observe(error, count=1):
            for _ in range(count):
                gate.observe([{'x': 320., 'y': 240.+error, 'kind': 'intermediate'}], 480, config)
        observe(0, 7)
        self.assertEqual(gate.state(config), 'stopped_intermediate')
        for error in (-8, 0, 0, 8, 0, 0):
            observe(error)
            self.assertEqual(gate.limit(.075, config)[0], 0.)
        observe(-8, 7)  # 轻微越线在9.6像素内，按到达处理，不倒车。
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        observe(4, 7)  # 未到的一侧仍保持3.84像素容差。
        self.assertGreater(gate.limit(.075, config)[0], 0.)
        observe(-10, 7)
        self.assertEqual(gate.limit(.075, config), (0., 'overshoot'))
        self.assertFalse(gate.resume(config))
        self.assertEqual(gate.station_id, 1)

    def test_stopped_bar_position_uses_fixed_axis_but_still_detects_real_motion(self):
        config = ControlConfig()
        points = [(320+.3*(y-240), y) for y in range(24, 460, 6)]
        mask = np.zeros((480, 640), np.uint8)
        cv2.polylines(mask, [np.array(points, np.int32)], False, 255, 18)
        cv2.line(mask, (220, 270), (420, 210), 255, 18)
        reference = detect_stop_markers(mask, points, config)[0]
        for shift in (-8, -4, 4, 8):
            noisy_points = [(x+shift, y) for x, y in points]
            markers = detect_stop_markers(mask, noisy_points, config, reference)
            self.assertTrue(markers)
            self.assertAlmostEqual(markers[0]['y'], reference['y'], delta=.5)
        # 横线真的移动时，仍根据新图像更新位置，不能锁死旧坐标来掩盖过冲。
        moved = cv2.warpAffine(mask, np.float32([[1, 0, 0], [0, 1, -8]]), (640, 480))
        markers = detect_stop_markers(moved, points, config, reference)
        self.assertTrue(markers)
        self.assertLess(markers[0]['y']-reference['y'], -6)

    def test_station_does_not_resume_using_marker_position_before_loss(self):
        config = ControlConfig()
        gate = StopMarkerGate()
        marker = {'x': 320., 'y': 240., 'kind': 'intermediate'}
        for _ in range(7):
            gate.observe([marker], 480, config)
        self.assertEqual(gate.state(config), 'stopped_intermediate')
        gate.observe([], 480, config)
        self.assertEqual(gate.limit(.075, config), (0., 'missing'))
        self.assertFalse(gate.resume(config))

    def test_mean_position_controls_forward_and_accepts_slight_overshoot(self):
        config = ControlConfig()
        gate = StopMarkerGate()
        def observe(y, count=7):
            for _ in range(count):
                gate.observe([{'x': 320., 'y': float(y), 'kind': 'intermediate'}], 480, config, now=0.)
        observe(246)
        self.assertEqual(gate.limit(.075, config), (config.stop_marker_creep_speed, 'approaching'))
        self.assertFalse(gate.position_confirmed)
        observe(242)
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        self.assertEqual(gate.station_id, 1)
        observe(246)
        self.assertGreater(gate.limit(.075, config)[0], 0.)  # 不再被历史停车标志锁住。
        observe(230.4)  # 负向容差边界：恰好越过图高2%。
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        observe(230)
        self.assertEqual(gate.limit(.075, config), (0., 'overshoot'))
        self.assertFalse(gate.resume(config))
        for bad in ({'stop_marker_average_frames': 2}, {'stop_marker_overshoot_tolerance_ratio': .004},
                    {'stop_marker_average_max_age_sec': 0}, {'stop_marker_overshoot_tolerance_ratio': .11}):
            with self.assertRaises(ValueError):
                replace(config, **bad).validate()

    def test_five_frame_mean_decides_stop_instead_of_single_frame(self):
        config = ControlConfig()
        gate = StopMarkerGate()
        values = []
        for error in (6, 6, -3, 6, 6):
            values.append(error)
            gate.observe([{'x': 320., 'y': 240.+error, 'kind': 'intermediate'}], 480, config)
            self.assertGreater(gate.limit(.075, config)[0], 0.)
            self.assertFalse(gate.position_confirmed)
        self.assertAlmostEqual(gate.snapshot(config)['error_ratio']*480, 4.2)
        for error in (-3, 6, -3, 6, -3, 6, -3):
            values.append(error)
            gate.observe([{'x': 320., 'y': 240.+error, 'kind': 'intermediate'}], 480, config)
            self.assertAlmostEqual(gate.error*480, sum(values[-5:])/5)
        self.assertEqual(gate.state(config), 'stopped_intermediate')
        self.assertEqual(gate.station_id, 1)
        self.assertEqual(gate.snapshot(config)['filter'], 'arithmetic_mean')

    def test_mean_stop_converges_from_either_side_in_both_image_directions(self):
        # 合成像素运动只用于验证反馈符号和收敛，不是实际相机的米制标定。
        for direction in ('down', 'up'):
            config = replace(ControlConfig(), image_forward_direction=direction, stop_marker_target_ratio=.46)
            forward = 1. if direction == 'down' else -1.
            for initial_error in (12., -8.):
                gate, error = StopMarkerGate(), initial_error
                for index in range(400):
                    now = index*.05
                    marker = {'x': 320., 'y': 480*.46+forward*error, 'kind': 'intermediate'}
                    gate.observe([marker], 480, config, now=now)
                    speed, state = gate.limit(.075, config)
                    self.assertGreaterEqual(speed, 0.)
                    error -= speed*1000*.05
                    if state == 'stopped_intermediate':
                        break
                self.assertEqual(state, 'stopped_intermediate', (direction, initial_error, error))
                self.assertEqual(speed, 0.)
                self.assertGreaterEqual(error, -.02*480)
                self.assertLessEqual(error, .008*480)
                self.assertEqual(gate.station_id, 1)

    def test_average_discards_old_samples_and_missing_frames_do_not_count_as_zero(self):
        config = ControlConfig()
        gate = StopMarkerGate()
        marker = {'x': 320., 'y': 246., 'kind': 'unknown'}
        for _ in range(5):
            gate.observe([marker], 480, config, now=0.)
        gate.observe([], 480, config, now=.1)
        self.assertEqual(gate.limit(.075, config), (0., 'missing'))
        self.assertAlmostEqual(gate.error*480, 6.)
        gate.observe([marker], 480, config, now=2.)
        self.assertEqual(len(gate.samples), 1)
        self.assertFalse(gate.position_confirmed)
        marker['y'] = 240.
        for _ in range(8):
            gate.observe([marker], 480, config, now=2.)
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_unknown'))
        self.assertFalse(gate.resume(config))

    def setUp(self):
        self.config = ControlConfig(forward_speed=0.05, image_forward_direction='up', require_bottom_connected=True, steering_sign=-1.0, vehicle_reference_y_ratio=1.05)

    def test_small_noise_and_continuous_deadband(self):
        for error, heading in ((0.02, 0.02), (-0.02, -0.02), (0.05, 0.04), (-0.05, -0.04)):
            self.assertEqual(line_command(observation(
                error_normalized=error, heading_error_rad=heading), self.config)[1], 0.0)
        # A just-visible error produces a tiny correction, not a step at the threshold.
        w = line_command(observation(error_normalized=self.config.error_deadband + 0.001), self.config)[1]
        self.assertAlmostEqual(abs(w), 0.00065)
        self.assertGreater(abs(line_command(observation(error_normalized=0.4), self.config)[1]), 0.1)

    def test_curve_geometry_does_not_confuse_straight_offset_with_bend(self):
        config = ControlConfig()
        for points in ([(400, y) for y in range(120, 270, 6)],
                       [(240, y) for y in range(120, 270, 6)]):
            geometry, reason = raw_tracking_geometry(points, 640, 480, config)
            self.assertIsNone(reason)
            self.assertAlmostEqual(geometry['curve_strength'], 0.0)
        tilted, _ = raw_tracking_geometry(
            [(319.5 + 0.5 * (y - 120), y) for y in range(120, 270, 6)], 640, 480, config)
        self.assertGreater(tilted['curve_strength'], 0.9)  # 出弯后的倾斜直线也要尽快对准。
        points = [(319.5 + 0.012 * (y - 120) ** 2, y) for y in range(120, 270, 6)]
        curved, reason = raw_tracking_geometry(points, 640, 480, config)
        self.assertIsNone(reason)
        self.assertGreater(curved['curve_strength'], 0.9)
        mirrored, _ = raw_tracking_geometry([(x, 479-y) for x, y in points], 640, 480,
            replace(config, image_forward_direction='up', vehicle_reference_y_ratio=1.05))
        self.assertAlmostEqual(mirrored['curve_strength'], curved['curve_strength'])
        self.assertAlmostEqual(mirrored['path_heading_rad'], curved['path_heading_rad'])

    def test_curve_follows_heading_without_fighting_allowed_center_offset(self):
        config = self.config
        status = observation(error_normalized=-0.2, heading_error_rad=0.3,
                             path_heading_rad=0.5, curve_strength=1.0)
        _, turning, reason = line_command(status, config)
        self.assertEqual(reason, 'tracking')
        _, old_turn, _ = line_command(dict(status, curve_strength=0.0), config)
        self.assertGreater(abs(turning), abs(old_turn))
        self.assertLess(turning, 0.0)  # image up、负符号约定：跟随右侧前方走向。
        centered = line_command(dict(status, error_normalized=0.0), config)
        offset = line_command(dict(status, error_normalized=-0.1), config)
        self.assertEqual(centered, offset)  # 弯道容差内不追回中心，也不因此减速。
        for strength in (0.0, 0.5, 1.0):
            result = line_command(dict(status, error_normalized=1.0,
                heading_error_rad=1.0, path_heading_rad=1.0, curve_strength=strength), config)
            limit = config.max_angular_speed + strength * (
                config.curve_max_angular_speed - config.max_angular_speed)
            self.assertLessEqual(abs(result[1]), limit)
        self.assertLessEqual(abs(smooth_yaw(0.0, 0.2, config, 0.02, 1.0)),
                             config.curve_angular_acceleration * 0.02)
        for bad in ({'curve_strength': math.nan}, {'curve_strength': True},
                    {'curve_strength': 1.1}, {'path_heading_rad': math.inf}):
            self.assertEqual(line_command(dict(status, **bad), config)[:2], (0.0, 0.0))
        for bad in ({'curve_detection_angle_rad': 0.0}, {'curve_error_deadband': 0.01},
                    {'curve_center_gain_factor': 1.1}, {'curve_angular_acceleration': 0.0}):
            with self.assertRaises(ValueError):
                replace(config, **bad).validate()

    def test_gentle_recentering_and_turn_limit(self):
        previous = replace(self.config, steering_gain=1.2, heading_gain=0.5,
                           error_deadband=0.03, heading_deadband_rad=0.035,
                           max_angular_speed=0.5, curve_slowdown=0.7)
        status = observation(error_normalized=0.2, heading_error_rad=0.1)
        v, w, reason = line_command(status, self.config)
        old_v, old_w, _ = line_command(status, previous)
        self.assertEqual(reason, 'tracking')
        self.assertLess(abs(w), abs(old_w))
        self.assertGreater(v, old_v)
        self.assertLessEqual(abs(line_command(observation(
            error_normalized=1.0, heading_error_rad=1.0), self.config)[1]), 0.25)
        self.assertAlmostEqual(slew(0.075, 0.025,
                                   self.config.linear_acceleration, 0.02), 0.0734)

    def test_steering_filter_rejects_jitter_and_tracks_curve(self):
        steady = noisy = 0.0
        noise = []
        for index in range(100):
            noisy = smooth_yaw(noisy, 0.05 if index % 2 == 0 else -0.05, self.config, 0.02)
            noise.append(abs(noisy))
            steady = smooth_yaw(steady, 0.2, self.config, 0.02)
        self.assertLess(max(noise), 0.01)
        self.assertAlmostEqual(steady, 0.2, places=3)
        self.assertLessEqual(abs(smooth_yaw(0.0, 0.5, self.config, 0.02)), 0.01)
        unfiltered = replace(self.config, steering_filter_time_sec=0.0)
        self.assertAlmostEqual(smooth_yaw(0.0, 0.005, unfiltered, 0.02), 0.005)
        for changes in ({'heading_deadband_rad': -0.1}, {'steering_filter_time_sec': 2.0}):
            with self.assertRaises(ValueError):
                replace(self.config, **changes).validate()

    def test_straight_and_direction(self):
        self.assertEqual(line_command(observation(), self.config), (0.05, 0.0, 'tracking'))
        self.assertLess(line_command(observation(error_normalized=0.5), self.config)[1], 0)
        self.assertGreater(line_command(observation(error_normalized=-0.5), self.config)[1], 0)
        config = replace(self.config, steering_sign=1.0)
        self.assertGreater(line_command(observation(error_normalized=0.5), config)[1], 0)

    def test_image_down_direction_and_heading(self):
        config = replace(self.config, image_forward_direction='down')
        self.assertGreater(line_command(observation(error_normalized=0.5), config)[1], 0)
        self.assertLess(line_command(observation(error_normalized=-0.5), config)[1], 0)
        self.assertGreater(line_command(observation(heading_error_rad=0.3), config)[1], 0)
        self.assertEqual(line_command(observation(heading_error_rad=1.5), config)[:2], (0, 0))
        self.assertGreater(line_command(observation(), config)[0], 0)

    def test_body_reference_and_forward_goal(self):
        points = [(450, 100), (400, 200), (300, 300), (200, 420)]
        geometry, reason = tracking_geometry(points, 640, 480, self.config)
        self.assertIsNone(reason)
        self.assertEqual(geometry['near_x'], 200)
        self.assertLess(geometry['target_y'], geometry['reference_y'])
        self.assertLess(geometry['error_normalized'], 0)
        # Far detector error is positive; near measured line is left. Correct
        # the chassis back left rather than steering from that far-only error.
        data = observation(error_normalized=0.4)
        data.update({k: geometry[k] for k in ('error_normalized', 'heading_error_rad')})
        self.assertGreater(line_command(data, self.config)[1], 0)
        shifted, _ = tracking_geometry(points, 640, 480,
                                       replace(self.config, vehicle_reference_x_ratio=0.4))
        self.assertGreater(shifted['error_normalized'], geometry['error_normalized'])

    def test_visible_geometry_limits(self):
        points = [(320, 10), (320, 160)]
        self.assertEqual(tracking_geometry(points, 640, 480, self.config)[1], 'near_line_missing')
        self.assertEqual(tracking_geometry(points, 640, 480,
                         replace(self.config, image_forward_direction='down'))[1], 'no_forward_line')
        down = replace(self.config, image_forward_direction='down', vehicle_reference_y_ratio=-0.05)
        geometry, reason = tracking_geometry(points, 640, 480, down)
        self.assertIsNone(reason)
        self.assertGreater(geometry['target_y'], geometry['reference_y'])
        self.assertEqual(tracking_geometry(points, None, None, self.config)[1], 'invalid_image_size')
        self.assertEqual(tracking_geometry([(math.nan, 100), (320, 420)], 640, 480, self.config)[1],
                         'insufficient_heading_points')

    def test_steering_sign_reverses_both_terms(self):
        status = observation(error_normalized=0.2, heading_error_rad=0.15)
        before = line_command(status, self.config)
        after = line_command(status, replace(self.config, steering_sign=1.0))
        self.assertEqual(before[0], after[0])
        self.assertAlmostEqual(before[1], -after[1])

    def test_horizontal_curve_requires_valid_geometry(self):
        curve = [(160, 420), (230, 350), (320, 270), (410, 170)]
        geometry, reason = tracking_geometry(curve, 640, 480, self.config)
        self.assertIsNone(reason)
        status = observation(orientation='horizontal', curve_geometry_valid=True)
        status.update({key: geometry[key] for key in ('error_normalized', 'heading_error_rad')})
        self.assertEqual(line_command(status, self.config)[2], 'tracking')
        status['curve_geometry_valid'] = False
        self.assertEqual(line_command(status, self.config)[:2], (0, 0))
        status['curve_geometry_valid'] = True
        self.assertEqual(line_command(status, replace(self.config, allow_curved_candidates=False))[:2], (0, 0))
        horizontal = [(150, 400), (300, 400), (450, 400)]
        self.assertEqual(tracking_geometry(horizontal, 640, 480, self.config)[1], 'no_forward_line')
        shallow = [(150, 400), (300, 410), (450, 420)]
        self.assertEqual(tracking_geometry(shallow, 640, 480, self.config)[1], 'insufficient_forward_span')

    def test_motion_arrow_matches_forward_and_turn(self):
        down = ControlConfig()
        straight = motion_preview(0.03, 0.0, down)
        self.assertAlmostEqual(straight[-1][0], 0)
        self.assertGreater(straight[-1][1], 0)
        up = replace(down, image_forward_direction='up', vehicle_reference_y_ratio=1.05)
        self.assertLess(motion_preview(0.03, 0.0, up)[-1][1], 0)
        self.assertGreater(motion_preview(0.03, 0.3, down)[-1][0], 0)
        self.assertLess(motion_preview(0.03, -0.3, down)[-1][0], 0)
        self.assertEqual(motion_preview(0.0, 0.0, down)[-1], (0.0, 0.0))
        # Preserve the corrected effective left/right mapping after relocating
        # the body reference from bottom/up/+1 to top/down/-1.
        status = observation(error_normalized=0.2, heading_error_rad=0.1)
        old = replace(down, image_forward_direction='up', steering_sign=1.0)
        self.assertEqual(line_command(status, old), line_command(status, down))

    def test_top_wheel_margin_allows_current_near_line(self):
        config = ControlConfig()
        visible = [(450, 144), (380, 240), (280, 340), (160, 459)]
        geometry, reason = tracking_geometry(visible, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(geometry['near_y'], 144)
        distant = [(430, 330), (380, 350), (280, 400), (160, 459)]
        self.assertEqual(tracking_geometry(distant, 640, 480, config)[1], 'near_line_missing')

    def test_near_ground_start_excludes_wheel_region(self):
        config = ControlConfig()
        current = [(321, 200), (260, 240), (180, 320), (63, 389)]
        geometry, reason = tracking_geometry(current, 640, 480, config)
        self.assertIsNone(reason)
        self.assertAlmostEqual(geometry['ground_start_y'], 119.75)
        self.assertAlmostEqual(geometry['near_gap_pixels'], 80.25)
        self.assertEqual(tracking_geometry(current, 640, 480,
                         replace(config, near_edge_margin_ratio=0.0))[1], 'near_line_missing')

    def test_visible_line_without_bottom_anchor(self):
        config = replace(self.config, require_bottom_connected=False)
        v, _, reason = line_command(observation(bottom_connected=False), config)
        self.assertGreater(v, 0.0)
        self.assertEqual(reason, 'tracking')
        self.assertEqual(line_command(observation(bottom_connected=False, lost=True), config)[:2], (0, 0))
        self.assertEqual(line_command(observation(bottom_connected=False, valid_row_fraction=0.01), config)[:2], (0, 0))

    def test_unusable_observations_stop(self):
        for values in (
            {'lost': True}, {'processing_over_budget': True},
            {'orientation': 'horizontal'}, {'bottom_connected': False},
            {'valid_row_fraction': 0.01}, {'error_normalized': math.nan},
            {'error_normalized': True}, {'error_normalized': 1.1},
        ):
            with self.subTest(values=values):
                self.assertEqual(line_command(observation(**values), self.config)[:2], (0, 0))
        self.assertEqual(line_command(None, self.config)[:2], (0, 0))

    def test_slowdown_and_limits(self):
        straight = line_command(observation(), self.config)[0]
        curve = line_command(observation(error_normalized=1.0), self.config)
        estimate = line_command(observation(target_estimated=True), self.config)[0]
        self.assertLess(curve[0], straight)
        self.assertLess(estimate, straight)
        self.assertLessEqual(abs(curve[1]), self.config.max_angular_speed)
        self.assertEqual(line_command(observation(error_normalized=0.5),
                                     replace(self.config, forward_speed=0.0))[:2], (0, 0))

    def test_invalid_parameters(self):
        self.config.validate()
        for values in ({'forward_speed': -1}, {'forward_speed': 1},
                       {'forward_speed': math.nan}, {'steering_sign': 0},
                       {'estimated_speed_factor': 2}, {'linear_acceleration': 0}, {'image_forward_direction': 'left'}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                replace(self.config, **values).validate()

    def test_acceleration_limit(self):
        self.assertAlmostEqual(slew(0.0, 0.1, 0.15, 0.02), 0.003)
        self.assertAlmostEqual(slew(0.01, -0.1, 1.0, 0.02), -0.01)


if __name__ == '__main__':
    unittest.main()
