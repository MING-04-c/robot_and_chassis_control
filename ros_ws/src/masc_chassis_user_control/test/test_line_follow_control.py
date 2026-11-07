#!/usr/bin/env python3
"""Unit checks for direction, lost-line stop, slowdown and parameter validation."""

import math
import sys
import unittest
import tempfile
import json

import cv2
import numpy as np
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from stop_marker import StopMarkerGate, detect_stop_markers
from recorded_route import RouteConfig, RouteRecorder, ReverseRouteFollower, ReverseFinishGuard, load_route
from line_follow_control import ControlConfig, line_command, motion_preview, tracking_geometry, slew, smooth_yaw  # noqa: E402
from line_follow_control import SteeringDirectionGuard, coordinated_velocity, straight_image_axis, reverse_visual_command
from reverse_visual import GroundMapping, near_line, strafe_command, aligned as visually_aligned, AlignmentSequencer
from start_alignment import StartLineAligner


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


class ParallelSlopeTests(unittest.TestCase):
    def test_vectorized_slope_matches_original_pairs(self):
        from statistics import median
        from stop_marker import _median_pair_slope
        rng = np.random.default_rng(42)
        for size in (0, 1, 3, 20, 70):
            points = rng.integers(-50, 100, (size, 2)).astype(float).tolist()
            for axis in (0, 1):
                for span in (3, 12):
                    pairs = [(a[1-axis]-b[1-axis])/(a[axis]-b[axis])
                             for i, a in enumerate(points) for b in points[i+1:]
                             if abs(a[axis]-b[axis]) >= span]
                    actual = _median_pair_slope(points, axis, span)
                    if pairs:
                        self.assertAlmostEqual(actual, median(pairs))
                    else:
                        self.assertIsNone(actual)


class StandardReplayTests(unittest.TestCase):
    def test_forward_arc_sign_and_pose_integration(self):
        from recorded_route import StandardRouteFollower
        for sign in (-1, 1):
            document = dict(points=[[math.sin(i*.02), sign*(1-math.cos(i*.02)), sign*i*.02]
                                   for i in range(51)])
            tracker = StandardRouteFollower(document, (0.,0.,0.), RouteConfig())
            pose = [0.,0.,0.]
            v,w,state = tracker.command(pose,.05)
            self.assertGreater(v,0.)
            self.assertGreater(sign*w,0.)
            for _ in range(800):
                v,w,state=tracker.command(pose,.05)
                pose[0]+=v*math.cos(pose[2])*.05
                pose[1]+=v*math.sin(pose[2])*.05
                pose[2]+=w*.05
                if state == 'standard_route_complete':
                    break
            self.assertEqual(state,'standard_route_complete')
            self.assertLess(math.dist(pose[:2],document['points'][-1][:2]),.025)

    def test_anchor_rotation_deviation_and_wrong_direction(self):
        from recorded_route import StandardRouteFollower
        document=dict(points=[[i*.01,0.,0.] for i in range(101)])
        tracker=StandardRouteFollower(document,(2.,3.,math.pi/2),RouteConfig())
        self.assertAlmostEqual(tracker.tracker.points[-1][0],2.)
        self.assertAlmostEqual(tracker.tracker.points[-1][1],4.)
        self.assertGreater(tracker.command((2.,3.,math.pi/2),.05)[0],0.)
        self.assertEqual(tracker.command((2.2,3.,math.pi/2),.05)[2], 'standard_route_deviation')
        bad=dict(points=[[0.,0.,0.],[-.1,0.,0.],[-.2,0.,0.]])
        self.assertEqual(StandardRouteFollower(bad,(0.,0.,0.),RouteConfig()).command((0.,0.,0.),.05)[2],
                         'standard_target_not_ahead')

    def test_recording_keeps_pose_telemetry_and_parameters(self):
        recorder=RouteRecorder(replace(RouteConfig(),sample_distance=.01),'odom','base_link')
        recorder.context={'speed':.03}
        for i in range(21):
            recorder.add((i*.01,0.,0.),stamp=10.+i*.3,velocity=(.03,0.,0.))
        recorder.finished=True
        data=recorder.document()
        self.assertEqual(len(data['points']),len(data['point_samples']))
        self.assertEqual(data['point_samples'][-1]['velocity'],[.03,0.,0.])
        self.assertEqual(data['recording_config']['speed'],.03)


class StartLineAlignmentTests(unittest.TestCase):
    @staticmethod
    def ready_aligner(config, mapping=None, **kwargs):
        aligner = StartLineAligner(config, mapping, **kwargs)
        aligner.command(-1., (0., 0., 0.), stopped=True)
        aligner.command(0., (0., 0., 0.), stopped=True)
        return aligner

    @staticmethod
    def frames(aligner, start, y=60., x=50., slope=0., source_time=None):
        for index in range(5):
            now = start+index*.03
            aligner.observe([dict(x=x, y=y, main_slope=slope)], 101, 101,
                            int(now*1e9), now, source_time=source_time)
        return now

    def test_filtered_image_error_direction_and_duplicate_frame(self):
        config = ControlConfig(image_forward_direction='down')
        aligner = self.ready_aligner(config, confirmation_frames=3, filter_frames=5)
        ys = (60., 61., 72., 60., 61.)
        for stamp, y in enumerate(ys, 1):
            self.assertTrue(aligner.observe([dict(x=50., y=y, main_slope=0.)], 101, 101,
                                            stamp, stamp*.1))
        self.assertAlmostEqual(aligner.position_error, .11)
        self.assertFalse(aligner.observe([dict(x=50., y=50., main_slope=0.)], 101, 101,
                                         5, .7), 'same source image must not add another sample')
        self.assertEqual(aligner.command(.7, (0., 0., 0.), line_ready=True, stopped=True),
                         'start_alignment_waiting_for_stable_image')
        self.assertAlmostEqual(aligner.position_error, .11)

        up_config = replace(config, image_forward_direction='up')
        up = self.ready_aligner(up_config)
        self.assertTrue(up.observe([dict(x=50., y=30., main_slope=0.)], 101, 101, 1, .1))
        self.assertAlmostEqual(up.position_error, .2)

    def test_calibrated_front_back_and_tangent_are_metric(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory)/'mapping.json'
            filename.write_text(json.dumps({
                'frame_id': 'base_link', 'units': 'm', 'image_size': [101, 101],
                'image_points_px': [[0, 0], [100, 0], [0, 100], [100, 100]],
                'ground_points_m': [[0, .5], [0, -.5], [1, .5], [1, -.5]],
            }))
            mapping = GroundMapping(filename)
            aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'), mapping,
                                         lateral_tolerance_m=.025)
            self.assertTrue(aligner.observe([dict(x=50., y=70., main_slope=.1)],
                                            101, 101, 1, .1))
            self.assertTrue(aligner.metric)
            self.assertAlmostEqual(aligner.position_error_m, .2, places=3)
            self.assertAlmostEqual(aligner.lateral_error, .02, places=3)
            self.assertLess(aligner.heading_error, 0.)
            self.assertEqual(aligner.snapshot()['geometry_mode'], 'ground_meters')
            # Once heading has been corrected, a fresh stationary window may
            # plan longitudinal motion with the tighter default yaw tolerance.
            aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'), mapping,
                                         lateral_tolerance_m=.025)
            for stamp in range(1, 6):
                aligner.observe([dict(x=50., y=70., main_slope=0.)], 101, 101,
                                stamp, stamp*.1)
            self.assertEqual(aligner.command(.5, (0., 0., 0.), stopped=True),
                             'start_alignment_moving_forward')
            self.assertAlmostEqual(aligner.step_distance, .015)
            self.assertIsNone(aligner.image_response_per_m)

            mismatched = self.ready_aligner(ControlConfig(image_forward_direction='down'), mapping)
            self.assertFalse(mismatched.observe([dict(x=50., y=70., main_slope=0.)],
                                                100, 101, 1, .1))
            self.assertTrue(mismatched.fault.startswith('start_alignment_failed_'))
            self.assertEqual(mismatched.command(.2, (0., 0., 0.)), mismatched.fault)

            behind = self.ready_aligner(ControlConfig(image_forward_direction='down'), mapping)
            behind.observe([dict(x=50., y=30., main_slope=0.)], 101, 101, 2, .2)
            self.assertAlmostEqual(behind.position_error_m, -.2, places=3)
            self.assertEqual(behind.command(.2, (0., 0., 0.), line_ready=True),
                             'start_alignment_waiting_for_stop')
            metric_yaw = self.ready_aligner(ControlConfig(image_forward_direction='down'), mapping)
            now = self.frames(metric_yaw, .1, y=50., slope=.3)
            self.assertEqual(metric_yaw.command(now, (0., 0., 0.), stopped=True),
                             'start_alignment_moving_yaw')
            self.assertLess(metric_yaw.velocity()[2], 0., 'metric heading is already in base_link')
            metric_side = self.ready_aligner(ControlConfig(image_forward_direction='down'), mapping)
            now = self.frames(metric_side, .1, x=40., y=50.)
            self.assertEqual(metric_side.command(now, (0., 0., 0.), stopped=True),
                             'start_alignment_moving_lateral')
            self.assertGreater(metric_side.velocity()[1], 0., 'positive ground Y requires left strafe')

    def test_command_is_bounded_and_alignment_requires_stable_fresh_frames(self):
        from line_follow_control import start_alignment_command
        config = ControlConfig(start_alignment_speed=.02, start_alignment_max_angular_speed=.08)
        marker = dict(x=90., y=80., main_slope=1.)
        v, w, state = start_alignment_command(marker, 101, 101, config, position_error=.2,
            position_tolerance=.01, lateral_error=.8, heading_error=.8)
        self.assertEqual(state, 'start_alignment_approaching')
        self.assertLessEqual(v, .02)
        self.assertLessEqual(abs(w), .08)
        v, _, state = start_alignment_command(marker, 101, 101, config, position_error=-.12,
            position_tolerance=.01, overshoot_tolerance=.04)
        self.assertEqual(state, 'start_alignment_reverse')
        self.assertGreaterEqual(v, -config.reverse_max_speed)

        aligner = self.ready_aligner(config, confirmation_frames=3, filter_frames=3)
        for stamp in range(1, 4):
            aligner.observe([dict(x=50., y=50., main_slope=0.)], 101, 101,
                            stamp, stamp*.1)
        self.assertTrue(aligner.visual_ready)
        self.assertEqual(aligner.command(.4, (0., 0., 0.), stopped=True),
                         'start_alignment_complete')

    def test_stop_observe_move_and_discard_delayed_frames(self):
        config = ControlConfig(image_forward_direction='down')
        aligner = StartLineAligner(config)
        aligner.reset(0., (0., 0., 0.))
        aligner.command(0., (0., 0., 0.), stopped=True)
        self.assertFalse(aligner.observe([dict(x=50., y=60.)], 101, 101, 1, .3))
        aligner.command(.61, (0., 0., 0.), stopped=True)
        self.assertFalse(aligner.observe([dict(x=50., y=60.)], 101, 101, 2, .62,
                                         source_time=.2), 'pipeline result captured before settling')
        now = self.frames(aligner, .7)
        self.assertEqual(aligner.command(now, (0., 0., 0.), stopped=True),
                         'start_alignment_moving_forward')
        self.assertAlmostEqual(aligner.step_distance, .005)
        self.assertGreater(aligner.velocity()[0], 0.)
        self.assertFalse(aligner.observe([dict(x=50., y=40.)], 101, 101,
                                         int(1e9), 1.), 'moving image cannot reverse the step')
        self.assertGreater(aligner.velocity()[0], 0.)
        self.assertEqual(aligner.command(1.1, (.005, 0., 0.), stopped=False),
                         'start_alignment_waiting_for_stop')
        self.assertEqual(aligner.velocity(), (0., 0., 0.))
        aligner.command(1.2, (.005, 0., 0.), stopped=True)
        aligner.command(1.81, (.005, 0., 0.), stopped=True)
        self.assertFalse(aligner.observe([dict(x=50., y=50.)], 101, 101,
                                         int(1.82e9), 1.82, source_time=1.1))
        now = self.frames(aligner, 1.9, y=50.)
        self.assertEqual(aligner.command(now, (.005, 0., 0.), stopped=True),
                         'start_alignment_complete')
        self.assertEqual(aligner.step_count, 1)

    def test_local_image_response_estimates_bounded_second_step(self):
        aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
        now = self.frames(aligner, .1)
        aligner.command(now, (0., 0., 0.), stopped=True)
        aligner.command(.6, (.005, 0., 0.), stopped=False)
        aligner.command(.7, (.005, 0., 0.), stopped=True)
        aligner.command(1.31, (.005, 0., 0.), stopped=True)
        now = self.frames(aligner, 1.4, y=59.)
        aligner.command(now, (.005, 0., 0.), stopped=True)
        self.assertAlmostEqual(aligner.image_response_per_m, 2.)
        self.assertAlmostEqual(aligner.step_distance, .015)
        self.assertFalse(aligner.snapshot()['metric'])

    def stopped_response(self, aligner, start, pose, y):
        aligner.command(start, pose, stopped=False)
        aligner.command(start+.1, pose, stopped=True)
        aligner.command(start+.71, pose, stopped=True)
        now = self.frames(aligner, start+.8, y=y)
        aligner.command(now, pose, stopped=True)

    def test_actual_odometry_distance_estimate_shrinks_near_target_both_directions(self):
        for direction in (1., -1.):
            aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
            now = self.frames(aligner, .1, y=50.+direction*3.)
            aligner.command(now, (0., 0., 0.), stopped=True)
            self.assertAlmostEqual(aligner.step_distance, direction*.005)
            # Actual stop includes run-on beyond the requested 5 mm.
            self.stopped_response(aligner, .6, (direction*.006, 0., 0.), 50.+direction*1.6)
            response = .014/.006
            self.assertAlmostEqual(aligner.image_response_per_m, response)
            self.assertAlmostEqual(aligner.snapshot()['estimated_remaining_m'], direction*.016/response)
            self.assertAlmostEqual(aligner.step_distance, direction*.8*(.016-.0075)/response)
            self.assertLess(abs(aligner.step_distance), .005)
            self.assertEqual(aligner.distance_estimate_source, 'local_response_tentative')

    def test_bad_response_discards_old_scale_and_returns_to_probe(self):
        for final_y, expected_reason in ((59., 'image_change_below_noise'),
                                         (60., 'wrong_or_invalid_response'),
                                         (52., 'inconsistent_response')):
            aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
            now = self.frames(aligner, .1)
            aligner.command(now, (0., 0., 0.), stopped=True)
            self.stopped_response(aligner, .6, (.005, 0., 0.), 59.)
            self.stopped_response(aligner, 2., (.020, 0., 0.), final_y)
            self.assertIsNone(aligner.image_response_per_m)
            self.assertEqual(aligner.response_reason, expected_reason)
            self.assertEqual(aligner.distance_estimate_source, 'probe_no_reliable_scale')
            self.assertIsNone(aligner.snapshot()['estimated_remaining_m'])
            expected_probe = .002 if abs(final_y-50.) <= 3. else .005
            self.assertAlmostEqual(aligner.step_distance, expected_probe)

    def test_response_rejects_cross_axis_motion_yaw_and_pixel_noise(self):
        for pose, y, reason in (((.005, .004, 0.), 59., 'motion_contaminated'),
                                ((.005, 0., .03), 59., 'motion_contaminated'),
                                ((.005, 0., 0.), 59.8, 'image_change_below_noise')):
            aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
            now = self.frames(aligner, .1)
            aligner.command(now, (0., 0., 0.), stopped=True)
            self.stopped_response(aligner, .6, pose, y)
            self.assertIsNone(aligner.image_response_per_m)
            self.assertEqual(aligner.response_reason, reason)

    def test_consistent_response_history_and_near_target_probe(self):
        aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
        now = self.frames(aligner, .1)
        aligner.command(now, (0., 0., 0.), stopped=True)
        self.stopped_response(aligner, .6, (.005, 0., 0.), 59.)
        self.stopped_response(aligner, 2., (.020, 0., 0.), 56.)
        self.assertAlmostEqual(aligner.image_response_per_m, 2.)
        self.assertEqual(aligner.snapshot()['response_sample_counts']['longitudinal'], 2)
        self.assertEqual(aligner.distance_estimate_source, 'local_response_confirmed')
        self.assertLessEqual(aligner.step_distance, aligner.config.start_alignment_step_distance)
        near = self.ready_aligner(ControlConfig(image_forward_direction='down'))
        now = self.frames(near, .1, y=51.6)
        near.command(now, (0., 0., 0.), stopped=True)
        self.assertAlmostEqual(near.step_distance, .002)
        self.assertIsNone(near.estimated_remaining_m)

    def test_lateral_near_target_probe_has_independent_minimum_and_braking_profile(self):
        for direction in (-1., 1.):
            config = ControlConfig(image_forward_direction='down')
            aligner = self.ready_aligner(config)
            # Reproduce the ~9.8% lateral error from the physical failure log.
            now = self.frames(aligner, .1, x=50.+4.9*direction, y=50.)
            aligner.command(now, (0., 0., 0.), stopped=True)
            self.assertAlmostEqual(aligner.step_lateral_m, direction*.005)
            v, initial, w = aligner.velocity()
            self.assertEqual((v, w), (0., 0.))
            expected = math.sqrt(2.*config.linear_acceleration*(.005-.001))
            self.assertAlmostEqual(initial, direction*expected)
            self.assertLessEqual(abs(initial), config.start_alignment_lateral_speed)
            aligner.command(now+.1, (0., direction*.002, 0.), stopped=False)
            self.assertAlmostEqual(aligner.velocity()[1], direction*config.start_alignment_min_lateral_speed)
            self.assertGreater(abs(initial), abs(aligner.velocity()[1]))
            aligner.command(now+.2, (0., direction*.005, 0.), stopped=False)
            self.assertEqual(aligner.velocity(), (0., 0., 0.))
            self.assertAlmostEqual(aligner.snapshot()['step_actual_m'], direction*.005)
            aligner.command(now+.3, (0., direction*.006, 0.), stopped=False)
            self.assertAlmostEqual(aligner.snapshot()['step_actual_m'], direction*.006)
            self.assertEqual(aligner.velocity(), (0., 0., 0.))

    def test_forward_and_reverse_braking_floor_stops_at_arrival(self):
        for direction in (-1., 1.):
            aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
            now = self.frames(aligner, .1, y=50.+10.*direction)
            aligner.command(now, (0., 0., 0.), stopped=True)
            initial = abs(aligner.velocity()[0])
            aligner.command(now+.1, (direction*.0038, 0., 0.), stopped=False)
            self.assertAlmostEqual(aligner.velocity()[0], direction*aligner.config.start_alignment_min_linear_speed)
            self.assertLess(abs(aligner.velocity()[0]), initial)
            self.assertEqual(aligner.velocity()[1:], (0., 0.))
            aligner.command(now+.2, (direction*.006, 0., 0.), stopped=False)
            self.assertEqual(aligner.velocity(), (0., 0., 0.))

    def test_lateral_floor_does_not_fake_progress_or_escalate_on_stall(self):
        aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
        now = self.frames(aligner, .1, x=54.9, y=50.)
        aligner.command(now, (0., 0., 0.), stopped=True)
        initial = aligner.velocity()
        for offset in (.1, 1., 3.9):
            aligner.command(now+offset, (0., 0., 0.), stopped=True)
            self.assertEqual(aligner.phase, 'moving')
            self.assertEqual(aligner.velocity(), initial)
            self.assertEqual(aligner.snapshot()['step_actual_m'], 0.)
        self.assertEqual(aligner.command(now+4.01, (0., 0., 0.), stopped=True),
                         'start_alignment_failed_step_timeout')
        self.assertEqual(aligner.velocity(), (0., 0., 0.))

    def test_lateral_step_and_configuration_allow_nominal_minimum_speed_braking(self):
        config = ControlConfig(image_forward_direction='down')
        aligner = self.ready_aligner(config)
        aligner.lateral_response_per_m = 10.
        aligner.response_samples['lateral'] = [10., 10.]
        now = self.frames(aligner, .1, x=51.1, y=50.)
        aligner.command(now, (0., 0., 0.), stopped=True)
        minimum = .001+config.start_alignment_min_lateral_speed**2/(2.*config.linear_acceleration)
        self.assertAlmostEqual(aligner.step_lateral_m, minimum)
        self.assertGreater(minimum, aligner.POSITION_STEP_TOLERANCE_M)
        self.assertLessEqual(minimum, config.start_alignment_step_distance)
        for changes in ({'start_alignment_min_lateral_speed': .01},
                        {'start_alignment_min_lateral_speed': .04},
                        {'start_alignment_step_distance': .003},
                        {'start_alignment_lateral_speed': .06, 'start_alignment_min_lateral_speed': .06}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(config, **changes).validate()

    def test_unstable_and_duplicate_images_never_start_motion(self):
        aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
        for index in range(20):
            now = .01+index*.02
            aligner.observe([dict(x=50., y=60.+3*(index%2))], 101, 101, index+1, now)
            aligner.command(now, (0., 0., 0.), stopped=True)
            self.assertEqual(aligner.velocity(), (0., 0., 0.))
        self.assertEqual(aligner.step_count, 0)
        self.assertFalse(aligner.observe([dict(x=50., y=50.)], 101, 101, 20, .45))
        aligner.command(2., (0., 0., 0.), stopped=True)
        self.assertFalse(aligner.visual_ready)
        self.assertEqual(len(aligner.samples), 0)

    def test_rolling_or_input_loss_invalidates_stationary_confirmation(self):
        aligner = self.ready_aligner(ControlConfig())
        now = self.frames(aligner, .1, y=50.)
        self.assertTrue(aligner.visual_ready)
        aligner.command(now, (0., 0., 0.), stopped=False)
        self.assertFalse(aligner.visual_ready)
        aligner.command(.4, (0., 0., 0.), stopped=True)
        self.assertNotEqual(aligner.state, 'start_alignment_complete')
        aligner.command(1.1, (0., 0., 0.), stopped=True)
        now = self.frames(aligner, 1.2, y=50.)
        aligner.command(now, (0., 0., 0.), stopped=True, odom_fresh=False)
        self.assertEqual(aligner.velocity(), (0., 0., 0.))
        self.assertFalse(aligner.visual_ready)

    def test_extra_confirmations_do_not_trigger_a_zero_correction_failure(self):
        aligner = self.ready_aligner(ControlConfig(), confirmation_frames=8)
        now = self.frames(aligner, .1, y=50.)
        aligner.command(now, (0., 0., 0.), stopped=True)
        self.assertIsNone(aligner.fault)
        self.assertEqual(aligner.step_count, 0)
        for index in range(3):
            now = .3+index*.03
            aligner.observe([dict(x=50., y=50.)], 101, 101, int(now*1e9), now)
        self.assertEqual(aligner.command(now, (0., 0., 0.), stopped=True),
                         'start_alignment_complete')

    def test_no_reverse_permission_and_invalid_step_parameters(self):
        config = ControlConfig(image_forward_direction='down', start_alignment_reverse_correction=False)
        aligner = self.ready_aligner(config)
        now = self.frames(aligner, .1, y=40.)
        self.assertEqual(aligner.command(now, (0., 0., 0.), stopped=True),
                         'start_alignment_failed_correction_unavailable')
        self.assertEqual(aligner.velocity(), (0., 0., 0.))
        for name, value in (('start_alignment_settle_sec', .01),
                            ('start_alignment_step_distance', .1)):
            with self.assertRaises(ValueError):
                replace(config, **{name: value}).validate()

    def test_step_stall_total_distance_and_reverse_limits_remain_latched(self):
        config = ControlConfig(image_forward_direction='down')
        aligner = self.ready_aligner(config)
        now = self.frames(aligner, .1)
        aligner.command(now, (0., 0., 0.), stopped=True)
        self.assertEqual(aligner.command(4.3, (0., 0., 0.), stopped=True),
                         'start_alignment_failed_step_timeout')
        self.assertEqual(aligner.velocity(), (0., 0., 0.))
        self.assertEqual(aligner.command(4.4, (.005, 0., 0.), stopped=True),
                         'start_alignment_failed_step_timeout')
        for pose, reason in (((.31, 0., 0.), 'distance'), ((-.11, 0., 0.), 'reverse')):
            aligner = StartLineAligner(config)
            aligner.reset(0., (0., 0., 0.))
            self.assertEqual(aligner.command(.1, pose), 'start_alignment_failed_'+reason+'_limit')
        aligner = StartLineAligner(config)
        aligner.reset(0.)
        self.assertEqual(aligner.command(46., (0., 0., 0.)), 'start_alignment_failed_timeout')

    def test_reverse_and_yaw_steps_use_odometry_not_new_visual_commands(self):
        config = ControlConfig(image_forward_direction='down')
        aligner = self.ready_aligner(config)
        now = self.frames(aligner, .1, y=40.)
        self.assertEqual(aligner.command(now, (0., 0., 0.), stopped=True),
                         'start_alignment_moving_reverse')
        self.assertLess(aligner.velocity()[0], 0.)
        aligner.command(.6, (-.005, 0., 0.), stopped=False)
        self.assertEqual(aligner.velocity(), (0., 0., 0.))
        aligner = self.ready_aligner(config)
        now = self.frames(aligner, .1, y=50., slope=.3)
        self.assertEqual(aligner.command(now, (0., 0., 0.), stopped=True),
                         'start_alignment_moving_yaw')
        self.assertEqual(aligner.velocity()[0], 0.)
        self.assertGreater(abs(aligner.velocity()[2]), 0.)
        self.assertLessEqual(abs(aligner.step_angle), .04)
        aligner.command(.6, (0., 0., aligner.step_angle), stopped=False)
        self.assertEqual(aligner.velocity(), (0., 0., 0.))

    def test_opposing_lateral_heading_errors_cannot_cancel(self):
        config = ControlConfig(image_forward_direction='down')
        aligner = self.ready_aligner(config)
        heading = -math.atan(config.start_alignment_lateral_gain*.08)/config.heading_gain
        now = self.frames(aligner, .1, x=54., y=50., slope=math.tan(heading))
        self.assertEqual(aligner.command(now, (0., 0., 0.), stopped=True),
                         'start_alignment_moving_yaw')
        v, vy, w = aligner.velocity()
        self.assertEqual((v, vy), (0., 0.))
        self.assertGreater(abs(w), .01)
        self.assertIsNone(aligner.fault)
        self.assertGreater(abs(aligner.step_angle), aligner.ANGLE_STEP_TOLERANCE_RAD)

    def test_logged_four_degree_departure_is_not_alignment_success(self):
        aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
        now = self.frames(aligner, .1, x=50., y=50., slope=math.tan(.07278785))
        self.assertEqual(aligner.command(now, (0., 0., 0.), stopped=True),
                         'start_alignment_moving_yaw')
        aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
        now = self.frames(aligner, .1, x=52.5, y=50.)
        self.assertEqual(aligner.command(now, (0., 0., 0.), stopped=True),
                         'start_alignment_moving_lateral')

    def test_heading_then_lateral_then_longitudinal_with_stopped_observations(self):
        config = ControlConfig(image_forward_direction='down')
        aligner = self.ready_aligner(config)
        now = self.frames(aligner, .1, x=60., y=60., slope=.3)
        aligner.command(now, (0., 0., 0.), stopped=True)
        self.assertEqual(aligner.step_axis, 'heading')
        v, vy, w = aligner.velocity()
        self.assertEqual((v, vy), (0., 0.))
        self.assertGreaterEqual(abs(w), config.start_alignment_min_angular_speed)
        self.assertGreater(abs(aligner.step_angle), aligner.ANGLE_STEP_TOLERANCE_RAD)
        yaw = aligner.step_angle
        pose = (0., 0., yaw)
        aligner.command(.6, pose, stopped=False)
        aligner.command(.7, pose, stopped=True)
        aligner.command(1.31, pose, stopped=True)
        now = self.frames(aligner, 1.4, x=60., y=60.)
        aligner.command(now, pose, stopped=True)
        self.assertEqual(aligner.step_axis, 'lateral')
        self.assertEqual(aligner.velocity()[::2], (0., 0.))
        side = aligner.step_lateral_m
        pose = (-math.sin(yaw)*side, math.cos(yaw)*side, yaw)
        aligner.command(1.8, pose, stopped=False)
        aligner.command(1.9, pose, stopped=True)
        aligner.command(2.51, pose, stopped=True)
        now = self.frames(aligner, 2.6, x=50., y=60.)
        aligner.command(now, pose, stopped=True)
        self.assertEqual(aligner.step_axis, 'longitudinal')
        self.assertEqual(aligner.velocity()[1:], (0., 0.))
        self.assertGreater(aligner.velocity()[0], 0.)

    def test_short_camera_dropout_preserves_dwell_and_requires_new_capture(self):
        aligner = self.ready_aligner(ControlConfig())
        now = self.frames(aligner, .1, y=50.)
        self.assertTrue(aligner.visual_ready)
        started, cutoff = aligner.stopped_since, aligner.accept_after
        aligner.command(.25, (0., 0., 0.), stopped=True, line_ready=False)
        self.assertEqual((aligner.stopped_since, aligner.accept_after), (started, cutoff))
        self.assertEqual(len(aligner.samples), 5)
        self.assertEqual(aligner.velocity(), (0., 0., 0.))
        self.assertEqual(aligner.command(.3, (0., 0., 0.), stopped=True),
                         'start_alignment_waiting_for_marker')
        self.assertFalse(aligner.observe([dict(x=50., y=50.)], 101, 101,
                                         int(.31e9), .31, source_time=.24))
        self.assertTrue(aligner.observe([dict(x=50., y=50.)], 101, 101,
                                        int(.32e9), .32, source_time=.32))
        self.assertEqual(aligner.command(.33, (0., 0., 0.), stopped=True),
                         'start_alignment_complete')

    def test_interrupted_motion_requires_full_stop_and_new_observation(self):
        aligner = self.ready_aligner(ControlConfig(image_forward_direction='down'))
        now = self.frames(aligner, .1)
        aligner.command(now, (0., 0., 0.), stopped=True)
        self.assertGreater(aligner.velocity()[0], 0.)
        aligner.command(.3, (.002, 0., 0.), stopped=False, line_ready=False)
        self.assertEqual(aligner.velocity(), (0., 0., 0.))
        self.assertEqual(len(aligner.samples), 0)
        aligner.command(.4, (.002, 0., 0.), stopped=True, line_ready=False)
        cutoff = aligner.accept_after
        aligner.command(.8, (.002, 0., 0.), stopped=True, line_ready=False)
        self.assertEqual(aligner.accept_after, cutoff)
        aligner.command(1.01, (.002, 0., 0.), stopped=True)
        self.assertFalse(aligner.observe([dict(x=50., y=50.)], 101, 101,
                                         int(1.02e9), 1.02, source_time=.9))
        self.assertEqual(aligner.step_count, 1)

    def test_lateral_direction_and_local_response_use_body_y_axis(self):
        for direction, steering, expected in (('down', -1., 1.), ('up', -1., -1.),
                                               ('down', 1., -1.)):
            with self.subTest(direction=direction, steering=steering):
                config = ControlConfig(image_forward_direction=direction, steering_sign=steering)
                aligner = self.ready_aligner(config)
                now = self.frames(aligner, .1, x=60., y=50.)
                aligner.command(now, (0., 0., 0.), stopped=True)
                self.assertGreater(expected*aligner.velocity()[1], 0.)
                side = aligner.step_lateral_m
                aligner.command(.6, (0., side, 0.), stopped=False)
                aligner.command(.7, (0., side, 0.), stopped=True)
                aligner.command(1.31, (0., side, 0.), stopped=True)
                now = self.frames(aligner, 1.4, x=59., y=50.)
                aligner.command(now, (0., side, 0.), stopped=True)
                self.assertAlmostEqual(aligner.lateral_response_per_m, 4.)
                self.assertIsNone(aligner.image_response_per_m)

    def test_minimum_yaw_target_exceeds_arrival_tolerance(self):
        aligner = self.ready_aligner(ControlConfig(image_forward_direction='down',
                                                   start_alignment_heading_tolerance_rad=.02))
        now = self.frames(aligner, .1, y=50., slope=math.tan(.0201))
        aligner.command(now, (0., 0., 0.), stopped=True)
        self.assertGreater(abs(aligner.step_angle), aligner.ANGLE_STEP_TOLERANCE_RAD)
        self.assertGreater(abs(aligner.velocity()[2]), .01)
        aligner.command(now+.01, (0., 0., 0.), stopped=True)
        self.assertEqual(aligner.phase, 'moving', 'unchanged odometry is not completion')

    def test_missing_marker_cannot_commit_previous_ready_samples(self):
        aligner = self.ready_aligner(ControlConfig())
        self.frames(aligner, .1, y=50.)
        self.assertTrue(aligner.visual_ready)
        self.assertFalse(aligner.observe([], 101, 101, int(.25e9), .25))
        self.assertEqual(aligner.command(.26, (0., 0., 0.), stopped=True),
                         'start_alignment_waiting_for_marker')
        self.assertEqual(aligner.velocity(), (0., 0., 0.))

    def test_expired_partial_window_and_zero_speed_cannot_move(self):
        aligner = self.ready_aligner(ControlConfig())
        self.frames(aligner, .1, y=50.)
        self.assertTrue(aligner.visual_ready)
        self.assertNotEqual(aligner.command(1.67, (0., 0., 0.), stopped=True),
                            'start_alignment_complete')
        self.assertGreater(len(aligner.samples), 0)
        self.assertLess(len(aligner.samples), aligner.filter_frames)
        self.assertFalse(aligner.visual_ready)
        for x, slope in ((50., .3), (60., 0.)):
            aligner = self.ready_aligner(ControlConfig(forward_speed=0.))
            self.frames(aligner, .1, x=x, y=50., slope=slope)
            aligner.command(.3, (0., 0., 0.), stopped=True)
            self.assertEqual(aligner.velocity(), (0., 0., 0.))
            self.assertEqual(aligner.step_count, 0)


class ChargingDockTests(unittest.TestCase):
    @staticmethod
    def feedback(error=0., heading=0., metric=False):
        return dict(error_normalized=error, lateral_error_m=error,
                    near_heading_rad=heading, metric=metric)

    def test_terminal_creep_avoids_driver_park_deadband_and_respects_speed_limits(self):
        from charging_dock import ChargingDock
        for marker_arrived, marker_error, direction in (
                (False, None, -1), (False, .2, -1), (True, 0., -1), (False, -.2, 1)):
            for speed_limit in (.03, .0115):
                config = replace(ControlConfig(), forward_speed=speed_limit)
                config.validate()
                dock = ChargingDock(RouteConfig())
                for frame in range(80):
                    v, vy, w, state = dock.command((0., 0., 0.), self.feedback(), frame,
                        frame*.1, config, True, marker_arrived, marker_error)
                    if state == 'reverse_visual_finish_tracking':
                        self.assertGreater(abs(v), .01, '会被驱动视为零速而驻车')
                        self.assertLessEqual(abs(v), speed_limit)
                        self.assertGreater(v*direction, 0.)
                        self.assertEqual((vy, w), (0., 0.))
                self.assertEqual(state, 'reverse_visual_finish_tracking')
        for speed in (0., .009):
            dock = ChargingDock(RouteConfig())
            result = dock.command((0., 0., 0.), self.feedback(), 1, 0.,
                replace(ControlConfig(), forward_speed=speed), True, False, .1)
            self.assertEqual(result[:3], (0., 0., 0.))
            self.assertEqual(result[3], 'speed_zero' if speed == 0. else
                             'charging_dock_speed_below_motion_threshold')
        for speed in (.008, .01, .031):
            with self.assertRaises(ValueError):
                replace(ControlConfig(), stop_marker_creep_speed=speed).validate()

    def test_align_before_motion_then_search_only_ten_cm(self):
        from charging_dock import ChargingDock
        dock, config = ChargingDock(RouteConfig()), ControlConfig()
        for frame in range(12):
            v, vy, w, state = dock.command((0., 0., 0.), self.feedback(.08, .2), frame,
                frame*.1, config, True, False, None)
            self.assertEqual(v, 0., '未摆正就后退')
            self.assertEqual(vy, 0., '朝向未摆正就横移')
        for frame in range(12, 24):
            result = dock.command((0., 0., 0.), self.feedback(), frame,
                                  frame*.1, config, True, False, None)
        self.assertLess(result[0], 0.)
        self.assertEqual(result[1:3], (0., 0.), '对齐后必须直线后退')
        result = dock.command((0., 0., 0.), self.feedback(), 24, 2.4, config, True, True, 0.)
        self.assertNotEqual(result[3], 'reverse_route_complete', '横线不能冒充已充电')
        result = dock.command((-.101, 0., 0.), self.feedback(), 25, 2.5, config, True, True, 0.)
        self.assertEqual(result, (0., 0., 0., 'charging_dock_search_limit'))
        self.assertEqual(dock.command((0., 0., 0.), self.feedback(), 26, 2.6,
                         config, True, True, 0.)[3], 'charging_dock_search_limit', '故障必须锁定')

    def test_small_offset_is_corrected_and_forward_reposition_is_bounded(self):
        from charging_dock import ChargingDock
        dock, config = ChargingDock(RouteConfig()), ControlConfig()
        results = [dock.command((0., 0., 0.), self.feedback(.05), i, i*.1,
                               config, True, False, None) for i in range(20)]
        self.assertTrue(any(abs(result[1]) > 0. for result in results), '小偏差不能卡在ready零速')
        dock = ChargingDock(RouteConfig())
        for i in range(20):
            result = dock.command((0., 0., 0.), self.feedback(), i, i*.1,
                                  config, True, False, -.2)
        self.assertGreater(result[0], 0.)
        result = dock.command((.051, 0., 0.), self.feedback(), 20, 2., config, True, False, -.2)
        self.assertEqual(result, (0., 0., 0., 'charging_dock_forward_limit'))

    def test_deviation_missing_line_timeout_and_unique_frames(self):
        from charging_dock import ChargingDock
        config = ControlConfig()
        for feedback in (self.feedback(.101, metric=True), self.feedback(.36), self.feedback(heading=.7)):
            dock = ChargingDock(RouteConfig())
            self.assertEqual(dock.command((0., 0., 0.), feedback, 1, 0., config, True, False, None)[3],
                             'charging_dock_alignment_out_of_range')
        dock = ChargingDock(RouteConfig())
        self.assertEqual(dock.command((0., 0., 0.), None, 1, 0., config, True, False, None)[3],
                         'charging_dock_line_unavailable')
        dock = ChargingDock(RouteConfig())
        for i in range(12):
            self.assertEqual(dock.command((0., 0., 0.), self.feedback(), 1, i*.1,
                             config, True, False, None)[0], 0., '同一帧不能重复累计到位')
        dock = ChargingDock(RouteConfig())
        dock.command((0., 0., 0.), self.feedback(), 1, 0., config, True, True, 0.)
        self.assertEqual(dock.command((0., 0., 0.), self.feedback(), 2, 31.,
                         config, True, True, 0.)[3], 'charging_dock_charge_not_detected')


class RecordedRouteTests(unittest.TestCase):
    def test_alignment_sequencer_hysteresis_stops_boundary_chatter(self):
        route = RouteConfig()
        sequence = AlignmentSequencer()
        feedback = dict(metric=False, error_normalized=.07, near_heading_rad=0.)
        for i in range(12):
            phase = sequence.update(feedback, route, i, i*.1)
        self.assertEqual(phase, 'ready')
        for i in range(12, 32):
            feedback['error_normalized'] = .087 if i % 2 else .079
            self.assertEqual(sequence.update(feedback, route, i, i*.1), 'ready')
        # 到终点仍用严格容差，迟滞不能把0.087当成最终合格；切换期间等待停稳。
        feedback['error_normalized'] = .087
        for i in range(32, 36):
            self.assertEqual(sequence.update(feedback, route, i, i*.1, terminal=True), 'settling')
        self.assertEqual(sequence.update(feedback, route, 40, 4., terminal=True, stopped=False), 'settling')
        self.assertEqual(sequence.update(feedback, route, 41, 4.1, terminal=True), 'lateral')
        vy, w = strafe_command(feedback, ControlConfig(), route)
        self.assertGreater(abs(vy), .01)
        self.assertEqual(w, 0.)
        feedback['error_normalized'] = -.087
        self.assertEqual(sequence.update(feedback, route, 42, 4.2, terminal=True), 'settling')
        for now in (4.3, 4.4, 4.5):
            self.assertEqual(sequence.update(feedback, route, 42, now, terminal=True), 'settling')
        self.assertEqual(sequence.axis_direction, 1, 'same frame falsely confirmed reverse steering')
        sequence.update(feedback, route, 43, 4.6, terminal=True)
        sequence.update(feedback, route, 44, 4.7, terminal=True)
        self.assertEqual(sequence.update(feedback, route, 45, 5.1, terminal=True, stopped=False), 'settling')
        self.assertEqual(sequence.update(feedback, route, 46, 5.2, terminal=True), 'lateral')

    def test_reverse_local_mask_axis_excludes_stop_bar_and_supports_ground_mapping(self):
        config, route = ControlConfig(), RouteConfig()
        mask = np.zeros((480, 640), np.uint8)
        cv2.line(mask, (340, 20), (340, 460), 255, 16)
        cv2.line(mask, (240, 160), (440, 160), 255, 16)
        points = [(340. if not 150 <= y <= 170 else 300., float(y)) for y in range(20, 461, 6)]
        feedback = near_line(points, mask, config)
        self.assertIsNotNone(feedback)
        self.assertLess(abs(feedback['near_heading_rad']), .001)
        self.assertGreater(feedback['removed_wide_rows'], 0)
        self.assertFalse(feedback['metric'])
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory)/'ground.json'
            data = dict(frame_id='base_link', units='m', image_size=[640, 480],
                        image_points_px=[[200, 100], [440, 100], [440, 400], [200, 400]],
                        ground_points_m=[[.1, -.24], [.1, .24], [.7, .24], [.7, -.24]])
            filename.write_text(json.dumps(data))
            mapping = GroundMapping(filename)
            feedback = near_line(points, mask, config, mapping)
            self.assertAlmostEqual(feedback['lateral_error_m'], .04, places=5)
            self.assertFalse(visually_aligned(feedback, route))
            vy, w = strafe_command(feedback, config, route)
            self.assertGreater(vy, 0.)
            self.assertLess(abs(w), 1e-6)
            changed = np.zeros((480, 800), np.uint8)
            changed[:, 332:349] = 255
            with self.assertRaises(ValueError):
                near_line(points, changed, config, mapping)
            data['ground_points_m'] = [[0., 0.]]*4
            filename.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                GroundMapping(filename)

    def test_holonomic_reverse_controls_axes_independently_and_converges(self):
        config, route = ControlConfig(), RouteConfig()
        feedback = dict(metric=False, error_normalized=.4, near_heading_rad=0.)
        self.assertGreater(strafe_command(feedback, config, route)[0], 0.)
        self.assertEqual(strafe_command(feedback, config, route)[1], 0.)
        self.assertEqual(strafe_command(dict(feedback, error_normalized=0., near_heading_rad=.2), config, route)[0], 0.)
        # 地面坐标局部运动学；车后方向只影响vx，横移纠偏符号不应倒置。
        for vx in (-.03, 0.):
            lateral, heading = .08, .18
            sequencer = AlignmentSequencer()
            for i in range(2500):
                data = dict(metric=True, lateral_error_m=lateral, near_heading_rad=heading)
                phase = sequencer.update(data, route, i//5, i*.02, terminal=i >= 2000)
                vy, w = strafe_command(data, config, route)
                vy = vy if phase == 'lateral' else 0.
                w = w if phase == 'heading' else 0.
                moving_vx = vx if phase == 'ready' and i < 2000 else 0.
                self.assertFalse(vy and w, 'two steering modes active simultaneously')
                lateral += (-vy+moving_vx*math.sin(heading))*.02
                heading -= w*.02
            self.assertLess(abs(lateral), route.visual_goal_lateral_m+.001)
            self.assertLess(abs(heading), route.finish_goal_heading+.001)

    def test_atomic_recording_validates_and_preserves_exact_endpoint(self):
        config = RouteConfig()
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory)/'route.json'
            recorder = RouteRecorder(config, 'odom', 'base_link')
            for i in range(34):
                recorder.add((i*.01, 0., 0.))
            recorder.save(filename)
            with self.assertRaises(ValueError):
                load_route(filename)  # 行驶中的周期备份不能当作已停稳的返回路线。
            recorder.finished = True
            recorder.save(filename)
            route = load_route(filename)
            self.assertEqual(route['points'][-1], [.33, 0., 0.])
            self.assertLess(len(route['points']), 34)
            self.assertFalse(list(Path(directory).glob('.route.json*')))
            # 新任务未走起来时先撤销旧记录，不能把旧路线误当作本次。
            RouteRecorder(config, 'odom', 'base_link').save(filename)
            with self.assertRaises(ValueError):
                load_route(filename)
            filename.write_text(json.dumps(dict(route, points=[[0, 0, 0], [1, 0, 0]])))
            with self.assertRaises(ValueError):
                load_route(filename)
            filename.write_text('{partial')
            with self.assertRaises(ValueError):
                load_route(filename)

    def test_reverse_pure_pursuit_converges_straight_curve_and_s_curve(self):
        config = replace(RouteConfig(), lookahead=.26)
        paths = [([(i*.01, 0., 0.) for i in range(101)]),
                 ([(math.sin(i*.01), 1-math.cos(i*.01), i*.01) for i in range(101)]),
                 ([(i*.01, .1*math.sin(math.pi*i*.01),
                    math.atan(.1*math.pi*math.cos(math.pi*i*.01))) for i in range(201)])]
        for points, speed in ((p, v) for p in paths for v in (.025, .05, .075, .08)):
            follower = ReverseRouteFollower({'points': points}, points[-1], config)
            x, y, yaw = points[-1]
            max_error = 0.
            for _ in range(6000):
                v, w, state = follower.command((x, y, yaw), speed)
                max_error = max(max_error, follower.diagnostics['tracking_error_m'])
                self.assertLessEqual(abs(w), abs(v)*config.max_curvature+1e-9)
                if state != 'reverse_route_tracking':
                    break
                self.assertLess(v, 0.)
                x += v*math.cos(yaw)*.02
                y += v*math.sin(yaw)*.02
                yaw += w*.02
            self.assertEqual(state, 'reverse_route_complete')
            self.assertLessEqual(math.dist((x, y), points[0][:2]), config.goal_tolerance)
            self.assertLess(max_error, .03)
            self.assertEqual(follower.command((x+1., y, yaw), .025), (0., 0., 'reverse_route_complete'))

    def test_logged_reverse_offset_can_enter_alignment_but_cannot_finish(self):
        route = RouteConfig()
        route.validate()
        feedback = dict(error_normalized=-.567, near_heading_rad=math.radians(-11.9))
        self.assertTrue(route.visual_entry_ok(feedback))
        self.assertFalse(route.finish_alignment_ok(feedback['error_normalized'], feedback['near_heading_rad']))
        self.assertFalse(route.visual_entry_ok(dict(feedback, error_normalized=-.76)))
        self.assertFalse(route.visual_entry_ok(dict(feedback, near_heading_rad=math.radians(46))))
        self.assertFalse(route.visual_entry_ok(dict(feedback, error_normalized=float('nan'))))
        with self.assertRaises(ValueError):
            replace(route, finish_entry_lateral=route.finish_exit_lateral).validate()
        vy, _ = strafe_command(feedback, ControlConfig(), route)
        self.assertLess(vy, -.01)

    def test_recorded_tight_bend_requires_matching_return_curvature(self):
        # 实车故障对应约20cm半径弯道；原2/m限制不能跟随去程已走过的5/m路径。
        radius = .2
        points = [(i*.01, 0., 0.) for i in range(81)]
        points += [(.8+radius*math.sin(t), -radius*(1-math.cos(t)), -t)
                   for t in (i*math.pi/200 for i in range(1, 101))]
        points += [(1., -.2-i*.01, -math.pi/2) for i in range(1, 81)]
        for cap, expected in ((2., 'reverse_target_not_behind'), (5., 'reverse_route_complete')):
            config = replace(RouteConfig(), lookahead=.26, max_curvature=cap)
            config.validate()
            tracker = ReverseRouteFollower({'points': points}, points[-1], config)
            x, y, yaw = points[-1]
            v = w = peak_error = 0.
            limited = False
            for _ in range(6000):
                target_v, target_w, state = tracker.command((x, y, yaw), .03)
                peak_error = max(peak_error, tracker.diagnostics['tracking_error_m'])
                limited |= tracker.diagnostics.get('curvature_limited', False)
                if state != 'reverse_route_tracking':
                    break
                v, w = coordinated_velocity(v, w, target_v, target_w, ControlConfig(), .02)
                self.assertLessEqual(abs(w), cap*abs(v)+1e-9)
                x += v*math.cos(yaw)*.02
                y += v*math.sin(yaw)*.02
                yaw += w*.02
            self.assertEqual(state, expected)
            if cap == 2.:
                self.assertTrue(limited)
                self.assertGreater(tracker.diagnostics['remaining_m'], .5)
            else:
                self.assertLess(peak_error, .03)

    def test_reverse_longer_lookahead_reduces_transient_correction(self):
        # 同一横向偏差与同一倒车速度，放长前瞻应减小瞬间转向，不能靠减速实现。
        points = [(i*.01, 0., 0.) for i in range(101)]
        commands = []
        for distance in (.20, .26):
            config = replace(RouteConfig(), lookahead=distance)
            tracker = ReverseRouteFollower({'points': points}, points[-1], config)
            v, w, state = tracker.command((1., .01, .01), .05)
            self.assertEqual(state, 'reverse_route_tracking')
            self.assertEqual(v, -.05)
            commands.append(w)
        self.assertEqual(math.copysign(1., commands[0]), math.copysign(1., commands[1]))
        self.assertLess(abs(commands[1]), abs(commands[0]))

    def test_return_speed_limit_and_approach_are_independent_of_lookahead(self):
        points = [(i*.01, 0., 0.) for i in range(101)]
        for lookahead in (.15, .30):
            config = replace(RouteConfig(), lookahead=lookahead)
            follower = ReverseRouteFollower({'points': points}, points[-1], config)
            for x, requested, expected in ((1., .05, -.05), (.5, .2, -.08),
                                            (.15, .05, -.05), (.05, .05, -.025),
                                            (.04, .003, -.003), (.04, 0., 0.)):
                # 连续里程计步进；大幅瞬移应被防跳段保护拒绝，不能用来测速度。
                while follower.previous[0]-x > .011:
                    follower.command((follower.previous[0]-.01, 0., 0.), requested)
                v, w, state = follower.command((x, 0., 0.), requested)
                self.assertEqual(state, 'reverse_route_tracking')
                self.assertAlmostEqual(v, expected)
                self.assertAlmostEqual(w, 0.)
            self.assertEqual(follower.command((.015, 0., 0.), .05),
                             (0., 0., 'reverse_route_complete'))
        for name in ('max_speed', 'approach_distance'):
            for invalid in (0., math.nan, math.inf, -1., True):
                with self.assertRaises(ValueError):
                    replace(RouteConfig(), **{name: invalid}).validate()

    def test_route_retains_optional_station_events_for_return_audit(self):
        recorder = RouteRecorder(RouteConfig(), 'odom', 'base_link')
        recorder.add((0., 0., 0.))
        recorder.add((.2, 0., 0.))
        recorder.record_station(dict(id=1, action='ignored_departure', kind='intermediate'))
        recorder.record_station(dict(id=2, action='arrived', kind='final'))
        recorder.finished = True
        with tempfile.TemporaryDirectory() as directory:
            filename = str(Path(directory)/'route.json')
            recorder.save(filename)
            self.assertEqual([event['id'] for event in load_route(filename)['station_events']], [1, 2])
            data = json.loads(Path(filename).read_text())
            data.pop('station_events')
            Path(filename).write_text(json.dumps(data))
            self.assertTrue(load_route(filename)['valid'])

    def test_remaining_straight_segment_rejects_bends_and_accepts_final_leg(self):
        config = RouteConfig()
        straight = ReverseRouteFollower({'points': [(i*.01, 0., 0.) for i in range(101)]},
                                         (1., 0., 0.), config)
        self.assertTrue(straight.remaining_is_straight())
        points = [(i*.01, 0., 0.) for i in range(51)]
        points += [(.5, i*.01, 0.) for i in range(1, 51)]
        bend = ReverseRouteFollower({'points': points}, (.5, .5, 0.), config)
        self.assertFalse(bend.remaining_is_straight())
        bend.progress = .6
        self.assertTrue(bend.remaining_is_straight())
        for name in ('finish_center_gain', 'finish_heading_gain', 'finish_center_tolerance', 'finish_straight_distance'):
            with self.assertRaises(ValueError):
                replace(config, **{name: math.nan}).validate()
        with self.assertRaises(ValueError):
            replace(config, finish_straight_distance=.2).validate()

    def test_mild_curve_feedforward_and_curved_corridor(self):
        config = RouteConfig()
        radius = 1.5
        points = [(radius*math.sin(i*.01/radius), radius*(1-math.cos(i*.01/radius)), i*.01/radius)
                  for i in range(101)]
        tracker = ReverseRouteFollower({'points': points}, points[-1], config)
        tracker.progress = tracker.arc[-1]-.6
        self.assertTrue(tracker.remaining_is_gentle())
        self.assertAlmostEqual(tracker.reference_curvature(), 1/radius, delta=.03)
        pose = (*tracker.at(tracker.progress), 0.)
        tracker.diagnostics['remaining_m'] = .6
        guard = ReverseFinishGuard(tracker, pose, config)
        self.assertIsNone(guard.check(pose), 'valid curved path was rejected by final tangent fence')
        self.assertLess(guard.diagnostics['lateral_error_m'], .001)
        for progress in (.5, .6, .7, .8, .9, tracker.arc[-1]):
            tracker.progress = progress
            self.assertIsNone(guard.check((*tracker.at(progress), 0.)))
        self.assertEqual(tracker.reference_curvature(), 0.)
        corner = [(i*.01, 0., 0.) for i in range(51)]+[(.5, i*.01, 0.) for i in range(1, 51)]
        sharp = ReverseRouteFollower({'points': corner}, corner[-1], config)
        self.assertFalse(sharp.remaining_is_gentle())

    def test_visual_finish_search_limit_and_latched_fault(self):
        config = RouteConfig()
        self.assertTrue(config.finish_alignment_ok(.07, math.radians(5.)))
        self.assertFalse(config.finish_alignment_ok(.513, 0.))
        self.assertFalse(config.finish_alignment_ok(0., math.radians(7.)))
        self.assertFalse(config.finish_alignment_ok(math.nan, 0.))
        self.assertGreater(config.finish_exit_lateral, .513)
        points = [(i*.01, 0., 0.) for i in range(101)]
        tracker = ReverseRouteFollower({'points': points}, points[-1], config)
        for index in range(100, 27, -1):
            tracker.command((index*.01, 0., 0.), .05)
        guard = ReverseFinishGuard(tracker, (.28, 0., 0.), config)
        previous_command = {'linear_x_mps': -.03, 'angular_z_radps': .02}
        guard.diagnostics.update(last_tracking_command=previous_command, control_terms_valid=True)
        self.assertIsNone(guard.check((0., 0., 0.)))
        self.assertFalse(guard.diagnostics['control_terms_valid'])
        self.assertEqual(guard.diagnostics['last_tracking_command'], previous_command)
        self.assertIsNone(guard.check((-.099, 0., 0.)))
        self.assertEqual(guard.check((-.101, 0., 0.)), 'reverse_finish_search_limit')
        self.assertEqual(guard.check((0., 0., 0.)), 'reverse_finish_search_limit')
        guard = ReverseFinishGuard(tracker, (.28, 0., 0.), config)
        self.assertEqual(guard.check((-.08, .08, 0.)), 'reverse_finish_search_limit')
        guard = ReverseFinishGuard(tracker, (.28, 0., 0.), config)
        self.assertEqual(guard.check((.28, .151, 0.)), 'reverse_finish_deviation')
        guard = ReverseFinishGuard(tracker, (.28, 0., 0.), config)
        for _ in range(30):
            guard.check((.27, 0., 0.))
            guard.check((.28, 0., 0.))
        self.assertEqual(guard.fault, 'reverse_finish_search_limit')
        guard.arrived = True
        self.assertEqual(guard.check((1., 1., 0.)), 'reverse_route_complete')
        for name in ('finish_speed', 'finish_distance', 'finish_extra_distance', 'finish_exit_lateral'):
            for invalid in (-1., math.nan, math.inf, True):
                with self.assertRaises(ValueError):
                    replace(config, **{name: invalid}).validate()
        self.assertEqual(config.finish_exit_lateral, .75)
        for invalid in (.49, .96):
            with self.assertRaises(ValueError):
                replace(config, finish_exit_lateral=invalid).validate()
        with self.assertRaises(ValueError):
            replace(config, finish_extra_distance=.101).validate()

    def test_alignment_after_odom_restart_and_closed_path_does_not_finish_early(self):
        config = RouteConfig()
        points = [(5.+i*.01, 3., 0.) for i in range(101)]
        follower = ReverseRouteFollower({'points': points}, (0., 0., math.pi/2), config)
        self.assertAlmostEqual(follower.points[-1][0], 0.)
        self.assertAlmostEqual(follower.points[-1][1], -1.)
        v, w, state = follower.command((0., 0., math.pi/2), .025)
        self.assertEqual(state, 'reverse_route_tracking')
        self.assertAlmostEqual(w, 0.)
        self.assertLess(v, 0.)
        points = [(math.sin(i*math.pi/100), 1-math.cos(i*math.pi/100), i*math.pi/100)
                  for i in range(201)]
        follower = ReverseRouteFollower({'points': points}, points[-1], config)
        self.assertEqual(follower.command(points[-1], .025)[2], 'reverse_route_tracking')
        for _ in range(20):
            follower.command(points[-1], .025)
        self.assertLess(follower.progress, .05)  # 原地及闭合线交叉不能跳到路线终点。

    def test_deviation_wrong_heading_and_invalid_route_parameters_stop(self):
        config = RouteConfig()
        points = [(i*.01, 0., 0.) for i in range(101)]
        follower = ReverseRouteFollower({'points': points}, points[-1], config)
        self.assertEqual(follower.command((1., .3, 0.), .025), (0., 0., 'reverse_route_deviation'))
        follower = ReverseRouteFollower({'points': points}, points[-1], config)
        self.assertEqual(follower.command((1., 0., math.pi), .025), (0., 0., 'reverse_target_not_behind'))
        for values in ({'lookahead': 0.}, {'max_curvature': 10.}, {'odom_timeout': float('nan')}):
            with self.assertRaises(ValueError):
                replace(config, **values).validate()


class ControlTests(unittest.TestCase):
    def test_small_heading_noise_does_not_reverse_wheel_demand(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        guard = SteeringDirectionGuard()
        for frame in range(60):
            heading = .09 if frame % 2 else -.09
            _, yaw, reason = line_command(observation(heading_error_rad=heading,
                                                     near_heading_rad=heading), config)
            self.assertEqual(reason, 'tracking')
            self.assertEqual(guard.update(yaw, config, frame, .05), 0.)
        # 确实需要转弯时，不会被小噪声死区挡住。
        self.assertGreater(guard.update(.2, config, 61, .05), 0.)

    def test_sustained_straight_offset_allows_gentle_centering(self):
        config = replace(ControlConfig(forward_ignore_start_marker=False), steering_filter_time_sec=0.)
        for side in (-1., 1.):
            guard = SteeringDirectionGuard()
            error = side*.055  # 日志持续约18px偏移；原始转向低于普通防摆动门槛。
            _, yaw, _ = line_command(observation(error_normalized=error, near_heading_rad=0.), config)
            self.assertLess(abs(yaw/config.forward_speed), config.steering_curvature_enter)
            for frame in range(config.straight_center_confirmation_frames-1):
                self.assertEqual(guard.update(yaw, config, frame, .05,
                                              error_normalized=error), 0.)
            for _ in range(20):
                self.assertEqual(guard.update(yaw, config, frame, .02,
                                              error_normalized=error), 0.)
            value = guard.update(yaw, config, frame+1, .05, error_normalized=error)
            self.assertGreater(side*value, 0.)
            self.assertTrue(guard.center_ready)
            # 回到直线允许带时撤销回中状态；同样需求在弯道不降低抗摆动门槛。
            guard.update(0., config, frame+2, .05, error_normalized=side*.03)
            self.assertFalse(guard.center_ready)
            for frame in range(20):
                self.assertEqual(guard.update(yaw, config, frame+100, .05,
                                              error_normalized=error, curve_strength=.8), 0.)
            self.assertFalse(guard.center_ready)

    def test_alternating_side_noise_never_enables_centering(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        guard = SteeringDirectionGuard()
        for frame in range(60):
            error = .065 if frame % 2 else -.065
            _, yaw, _ = line_command(observation(error_normalized=error, near_heading_rad=0.), config)
            self.assertEqual(guard.update(yaw, config, frame, .05, error_normalized=error), 0.)
            self.assertFalse(guard.center_ready)
        for bad in ({'straight_center_curvature_enter': 0.},
                    {'straight_center_curvature_enter': .4},
                    {'straight_center_confirmation_frames': True},
                    {'straight_center_confirmation_frames': 2}):
            with self.assertRaises(ValueError):
                replace(config, **bad).validate()

    def test_reverse_confirmation_counts_new_images_and_survives_zero_gap(self):
        config = replace(ControlConfig(forward_ignore_start_marker=False), steering_filter_time_sec=0.)
        guard = SteeringDirectionGuard()
        scale = config.forward_speed
        self.assertEqual(guard.update(scale*.8, config, 0, .02), .8)
        guard.update(0., config, 1, .02)
        self.assertEqual(guard.update(-scale*.8, config, 2, .02), 0.)
        for _ in range(20):
            self.assertEqual(guard.update(-scale*.8, config, 2, .02), 0.)
        self.assertEqual(guard.pending_frames, 1)
        self.assertEqual(guard.update(-scale*.8, config, 3, .02), 0.)
        self.assertAlmostEqual(guard.update(-scale*.8, config, 4, .02), -.8)
        # 大幅反向弯在两张新图后响应，不受弱纠偏的三帧延迟。
        self.assertEqual(guard.update(scale*2., config, 5, .02), 0.)
        self.assertAlmostEqual(guard.update(scale*2., config, 6, .02), 2.)

    def test_weak_alternating_turn_with_zero_does_not_switch_direction(self):
        config = replace(ControlConfig(forward_ignore_start_marker=False), steering_filter_time_sec=0.)
        guard = SteeringDirectionGuard()
        guard.update(.06, config, 0, .05)
        for frame in range(1, 30):
            target = -.06 if frame % 2 else 0.
            self.assertGreaterEqual(guard.update(target, config, frame, .05), 0.)
        for updates in ({'steering_curvature_enter': .1}, {'steering_reverse_frames': True},
                        {'steering_reverse_frames': 1}, {'steering_fast_reverse_curvature': .2}):
            with self.assertRaises(ValueError):
                replace(config, **updates).validate()

    def test_coordinated_acceleration_preserves_turn_ratio_in_both_directions(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        for sign in (-1., 1.):
            v = w = 0.
            for target in (.075, .025, .008):
                for _ in range(80):
                    old_v, old_w = v, w
                    v, w = coordinated_velocity(v, w, sign*target, sign*target*2., config, .02)
                    self.assertAlmostEqual(w/v, 2.)
                    self.assertLessEqual(abs(v-old_v), config.linear_acceleration*.02+1e-10)
                    self.assertLessEqual(abs(w-old_w), config.angular_acceleration*.02+1e-10)
            self.assertEqual(coordinated_velocity(v, w, 0., 1., config, .02), (0., 0.))

    def test_straight_position_error_is_not_counted_again_as_heading(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        status = observation(error_normalized=.3, heading_error_rad=.4)
        old = line_command(status, config)[1]
        new = line_command(dict(status, near_heading_rad=0.), config)[1]
        self.assertAlmostEqual(new, math.atan(config.steering_gain*(.3-config.error_deadband)))
        self.assertLess(abs(new), abs(old))

    def test_adaptive_preview_is_continuous_and_keeps_straight_preview(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        straight, _ = raw_tracking_geometry([(320, y) for y in range(120, 470, 6)], 640, 480, config)
        self.assertEqual(straight['effective_lookahead_ratio'], config.lookahead_distance_ratio)
        for slope in np.linspace(.01, .45, 45):
            geometry, reason = raw_tracking_geometry(
                [(320+slope*(y-120), y) for y in range(120, 470, 6)], 640, 480, config)
            self.assertIsNone(reason)
            self.assertAlmostEqual(geometry['effective_lookahead_ratio'], config.lookahead_distance_ratio)
        previous = straight['effective_lookahead_ratio']
        for bend in np.linspace(.0001, .012, 120):
            geometry, reason = raw_tracking_geometry(
                [(320+bend*(y-120)**2, y) for y in range(120, 270, 6)], 640, 480, config)
            self.assertIsNone(reason)
            ratio = geometry['effective_lookahead_ratio']
            self.assertGreaterEqual(ratio, config.curve_lookahead_distance_ratio)
            self.assertLessEqual(ratio, previous+1e-9)
            self.assertLess(abs(ratio-previous), .02)
            previous = ratio
        self.assertAlmostEqual(previous, config.curve_lookahead_distance_ratio)

    def test_final_visual_axis_uses_measured_straight_line_and_rejects_curve(self):
        config = replace(ControlConfig(), drive_direction='reverse')
        for direction in ('up', 'down'):
            config = replace(config, image_forward_direction=direction,
                             vehicle_reference_y_ratio=1.05 if direction == 'up' else -.05)
            points = [(330+.1*(y-240), y) for y in range(20, 460, 6)]
            geometry, reason = raw_tracking_geometry(points, 640, 480, config)
            self.assertIsNone(reason)
            axis = straight_image_axis(points, 640, 480, geometry, config)
            self.assertIsNotNone(axis)
            self.assertAlmostEqual(axis['near_heading_rad'], math.atan(.1)*(1 if direction == 'down' else -1))
            self.assertLess(axis['fit_residual_px'], .1)
            self.assertIsNone(straight_image_axis(points, 640, 480,
                dict(geometry, bridged_gaps=[{}]), config))
        config = replace(config, image_forward_direction='down', vehicle_reference_y_ratio=-.05)
        points = [(320+.003*(y-200)**2, y) for y in range(110, 460, 6)]
        geometry, reason = raw_tracking_geometry(points, 640, 480, config)
        self.assertIsNone(reason)
        self.assertIsNone(straight_image_axis(points, 640, 480, geometry, config))

    def test_final_visual_centering_improves_offset_in_short_reverse_model(self):
        route = RouteConfig()
        config = replace(ControlConfig(), drive_direction='reverse', forward_speed=route.finish_speed,
                         reverse_max_speed=route.finish_speed, steering_gain=route.finish_center_gain,
                         reverse_heading_gain=route.finish_heading_gain,
                         error_deadband=route.finish_center_tolerance, heading_deadband_rad=.02,
                         max_angular_speed=.06, curve_max_angular_speed=.06)
        lateral, yaw, v, w, curvature = .08, .05, 0., 0., 0.
        # 简化相机投影及车体模型，仅验证反馈趋势，不是实车精度结论。
        for _ in range(1000):
            status = observation(error_normalized=(lateral-.15*yaw)/.3,
                                 heading_error_rad=-yaw, near_heading_rad=-yaw, curve_strength=0.)
            target_v, target_w, reason = line_command(status, config)
            self.assertEqual(reason, 'tracking')
            target = max(-route.max_curvature, min(route.max_curvature, target_w/target_v))
            curvature += .02/(.25+.02)*(target-curvature)
            v, w = coordinated_velocity(v, w, target_v, target_v*curvature, config, .02)
            lateral -= v*math.sin(yaw)*.02
            yaw += w*.02
            self.assertLessEqual(abs(w), .06+1e-9)
        self.assertLess(abs(lateral), .025)
        self.assertLess(abs(yaw), .08)

    def test_front_camera_reverse_separates_heading_from_lateral_correction(self):
        for direction in ('up', 'down'):
            forward = replace(ControlConfig(forward_ignore_start_marker=False), image_forward_direction=direction)
            reverse = replace(forward, drive_direction='reverse')
            side = observation(error_normalized=.4, near_heading_rad=0.)
            vf, wf, _ = line_command(side, forward)
            vr, wr, reason = line_command(side, reverse)
            self.assertGreater(vf, 0.)
            self.assertLess(vr, 0.)
            self.assertLessEqual(abs(vr), reverse.reverse_max_speed)
            self.assertEqual(reason, 'tracking')
            self.assertAlmostEqual(wr, -wf)
            aligned = observation(heading_error_rad=.2, near_heading_rad=.2)
            self.assertGreater(line_command(aligned, forward)[1]*line_command(aligned, reverse)[1], 0.)
            self.assertEqual(line_command(observation(), reverse)[:2], (0., 0.))
            for key, value in (('lost', True), ('valid_row_fraction', 0.), ('processing_over_budget', True)):
                self.assertEqual(line_command(dict(side, **{key: value}), reverse)[:2], (0., 0.))

    def test_logged_final_reverse_offset_and_heading_correct_together(self):
        route = RouteConfig()
        legacy = replace(ControlConfig(), drive_direction='reverse', reverse_steering_sign=-1.)
        control = replace(legacy, forward_speed=.03, reverse_max_speed=.03,
                          reverse_steering_sign=route.finish_lateral_sign,
                          steering_gain=route.finish_center_gain, reverse_heading_gain=route.finish_heading_gain,
                          error_deadband=route.finish_center_tolerance, heading_deadband_rad=.02,
                          max_angular_speed=.06)
        # 现场记录：线偏左且朝向正偏，两项应同向纠正，不能继续输出负角速度。
        for error, heading_deg, reference in ((-.136, 6.3, .125), (-.199, 8.1, .247), (-.295, 11.7, -.105)):
            data = observation(error_normalized=error, near_heading_rad=math.radians(heading_deg))
            v, w, reason = reverse_visual_command(data, control)
            self.assertEqual(reason, 'tracking')
            self.assertLess(v, 0.)
            self.assertGreater(w, 0.)
            curvature = max(-route.max_curvature, min(route.max_curvature, reference+w/v))
            self.assertGreater(v*curvature, 0.)
        self.assertEqual(legacy.reverse_steering_sign, -1.)
        for invalid in (0., 2., True, math.nan, '1'):
            with self.assertRaises(ValueError):
                replace(route, finish_lateral_sign=invalid).validate()

    def test_visual_reverse_calibration_does_not_invert_heading_feedback(self):
        # 现场日志：负偏移持续放大时，不能因横向项更大而在车头已斜向另一侧后继续正转。
        cfg = replace(ControlConfig(), drive_direction='reverse', reverse_heading_gain=.8,
                      reverse_max_speed=.03, error_deadband=.02, heading_deadband_rad=.02,
                      reverse_steering_sign=1., max_angular_speed=.06)
        for error, angle in ((-.590, -11.8), (-.696, -20.5)):
            data = observation(error_normalized=error, near_heading_rad=math.radians(angle))
            self.assertLess(reverse_visual_command(data, cfg)[1], 0.)
        for direction in ('up', 'down'):
            cfg = replace(ControlConfig(), drive_direction='reverse', image_forward_direction=direction)
            for heading in (-.2, .2):
                data = observation(near_heading_rad=heading, heading_error_rad=heading)
                first = reverse_visual_command(data, replace(cfg, reverse_steering_sign=1.))[1]
                second = reverse_visual_command(data, replace(cfg, reverse_steering_sign=-1.))[1]
                self.assertEqual(first, second)
                forward = line_command(data, replace(cfg, drive_direction='forward'))[1]
                self.assertGreater(first*forward, 0.)
            for side in (-.2, .2):
                data = observation(error_normalized=side, near_heading_rad=0.)
                first = reverse_visual_command(data, replace(cfg, reverse_steering_sign=1.))[1]
                second = reverse_visual_command(data, replace(cfg, reverse_steering_sign=-1.))[1]
                self.assertAlmostEqual(first, -second)
            data = observation(lost=True, near_heading_rad=0.)
            self.assertEqual(reverse_visual_command(data, cfg)[:2], (0., 0.))

    def test_visual_reverse_mild_curve_and_independent_lateral_calibration_converge(self):
        # 简化车体与图像投影；检验反馈分项符号和小弯趋势，不承诺实车精度。
        route = RouteConfig()
        for calibration in (-1., 1.):
            cfg = replace(ControlConfig(), drive_direction='reverse', forward_speed=.03,
                          reverse_max_speed=.03, reverse_steering_sign=calibration,
                          steering_gain=.65, reverse_heading_gain=.8, error_deadband=.02,
                          heading_deadband_rad=.02, max_angular_speed=.06)
            lateral, yaw_error, v, w, curvature = .06, .08, 0., 0., 0.
            reference = .6
            for _ in range(1500):
                data = observation(error_normalized=calibration*(lateral-.15*yaw_error)/.3,
                                   near_heading_rad=-yaw_error, heading_error_rad=-yaw_error)
                target_v, correction, reason = reverse_visual_command(data, cfg)
                self.assertEqual(reason, 'tracking')
                target = max(-2., min(2., reference+correction/target_v))
                curvature += .02/(.25+.02)*(target-curvature)
                v, w = coordinated_velocity(v, w, target_v, target_v*curvature, cfg, .02)
                lateral -= v*math.sin(yaw_error)*.02
                yaw_error += (w-reference*v)*.02
                self.assertLessEqual(abs(w), .06+1e-8)
            self.assertLess(abs(lateral), .025)
            self.assertLess(abs(yaw_error), .08)

    def test_reverse_device_sign_changes_yaw_only(self):
        # 标准车身运动律与实车方向标定分开验证，不能用输出翻转证明实车收敛。
        for image_direction in ('up', 'down'):
            for side, heading in ((.2, 0.), (0., .2), (-.2, -.1)):
                data = observation(error_normalized=side, near_heading_rad=heading)
                for direction in ('forward', 'reverse'):
                    standard = replace(ControlConfig(forward_ignore_start_marker=False), image_forward_direction=image_direction,
                                       drive_direction=direction, reverse_steering_sign=1.)
                    corrected = replace(standard, reverse_steering_sign=-1.)
                    v, w, reason = line_command(data, standard)
                    cv, cw, cr = line_command(data, corrected)
                    self.assertEqual((v, reason), (cv, cr))
                    self.assertAlmostEqual(cw, -w if direction == 'reverse' else w)
        for bad in (0., 2., True, 'minus'):
            with self.assertRaises(ValueError):
                replace(ControlConfig(forward_ignore_start_marker=False), reverse_steering_sign=bad).validate()

    def test_front_camera_reverse_straight_kinematics_converges(self):
        # 简化平面模型：半图宽对应0.3m，前置观测点领先0.15m；只检验符号和稳定趋势。
        # 这不是相机实测投影，也不代表已验证实车倒车精度。
        for drive in ('forward', 'reverse'):
            config = replace(ControlConfig(forward_ignore_start_marker=False), drive_direction=drive, forward_speed=.025)
            lateral, yaw, previous_w = .08, .05, 0.
            previous_v = 0.
            guard = SteeringDirectionGuard()
            for index in range(4000):
                heading = -yaw
                error = (lateral-.15*yaw)/.3
                v, wanted, reason = line_command(observation(error_normalized=error,
                    heading_error_rad=heading, near_heading_rad=heading), config)
                self.assertEqual(reason, 'tracking')
                curvature = guard.update(wanted, config, int(index*.02/.05), .02,
                                         error_normalized=error)
                previous_v, previous_w = coordinated_velocity(previous_v, previous_w,
                    v, curvature*v, config, .02)
                lateral -= previous_v*math.sin(yaw)*.02
                yaw += previous_w*.02
            self.assertLess(abs(lateral), .025, drive)
            self.assertLess(abs(yaw), .08, drive)

    def test_reverse_can_leave_initial_station_and_stop_at_next(self):
        for image_direction in ('up', 'down'):
            config = replace(ControlConfig(forward_ignore_start_marker=False), drive_direction='reverse',
                             image_forward_direction=image_direction)
            sign = -1. if image_direction == 'down' else 1.
            gate = StopMarkerGate()
            def bar(error):
                return {'x': 320., 'y': 480*(.5+sign*error), 'kind': 'intermediate'}
            gate.observe([bar(-.01)], 480, config, now=0.)
            self.assertTrue(gate.startup_ignored)
            self.assertEqual(gate.limit(-.025, config), (-.025, 'searching'))
            # 两帧漏检后回来仍是出发横线，不能立即重新停车。
            for i in range(1, 4):
                gate.observe([], 480, config, now=i*.05)
            gate.observe([bar(-.02)], 480, config, now=.2)
            self.assertEqual(gate.limit(-.025, config), (-.025, 'searching'))
            for frame in range(3):
                gate.observe([bar(-.10)], 480, config, now=.3+frame*.02)
                self.assertTrue(gate.startup_ignored)
                self.assertEqual(gate.limit(-.025, config), (-.025, 'searching'))
            gate.observe([bar(-.16)], 480, config, now=.4)
            self.assertTrue(gate.startup_ignored)
            gate.observe([bar(-.49)], 480, config, now=.41)
            self.assertFalse(gate.startup_ignored)
            # 解除后重复出现的这根已离站横线也不应再触发停车。
            gate.observe([bar(-.16)], 480, config, now=.42)
            self.assertEqual(gate.limit(-.025, config), (-.025, 'searching'))
            self.assertEqual(gate.station_id, 0)
            # 新站接近及到位仍照常停车计数。
            for index, error in enumerate((.15, .1, .05, .02, 0.)):
                for frame in range(7):
                    gate.observe([bar(error)], 480, config, now=.5+index*.2+frame*.02)
            self.assertEqual(gate.limit(-.025, config), (0., 'stopped_intermediate'))
            self.assertEqual(gate.station_id, 1)
            self.assertTrue(gate.resume(config))
            self.assertTrue(gate.startup_checked)  # 途中继续不会重新开启出发横线忽略窗口。

    def test_departure_dropout_keeps_identity_and_station_counts(self):
        config = replace(ControlConfig(), drive_direction='reverse')
        gate = StopMarkerGate()
        def bar(y):
            return {'x': 320., 'y': float(y), 'kind': 'intermediate'}
        gate.observe([bar(240)], 480, config, now=0.)
        gate.record_motion(-.05, 1., config)  # 现场仅退约5cm，出发线反复漏检。
        for index in range(20):
            gate.observe([] if index%2 else [bar(270)], 480, config, now=.1+index*.05)
            self.assertEqual(gate.limit(-.05, config)[1], 'searching')
        self.assertEqual(gate.ledger.counts['encountered'], 1)
        self.assertEqual(gate.ledger.counts['ignored'], 1)
        self.assertEqual(gate.ledger.counts['arrived'], 0)
        gate.observe([bar(330)], 480, config, now=1.2)
        for _ in range(5):
            gate.observe([bar(330)], 480, config, now=1.3)
        self.assertEqual(gate.ledger.counts['passed'], 1)
        gate.observe([bar(475)], 480, config, now=1.4)
        for y in (150, 180, 210, 240):
            for _ in range(5):
                gate.observe([bar(y)], 480, config, now=1.5)
        self.assertEqual(gate.ledger.counts['encountered'], 2)
        self.assertEqual(gate.ledger.counts['arrived'], 1)
        self.assertTrue(gate.resume(config))
        self.assertEqual(gate.ledger.counts['resumed'], 1)
        self.assertEqual(gate.ledger.counts['ignored'], 1)
        gate.record_motion(-.05, .2, config)
        gate.observe([bar(330)], 480, config, now=1.6)
        self.assertEqual(gate.ledger.counts['passed'], 2)
        snapshot = gate.snapshot(config)
        self.assertEqual([row['id'] for row in snapshot['station_history']], [1, 2])
        self.assertEqual([e['action'] for e in gate.ledger.pending].count('ignored_departure'), 1)
        held = StopMarkerGate()
        held.observe([bar(240)], 480, config, now=0.)
        held.record_odometry_distance(0.)
        held.record_motion(-.05, 6., config)
        held.observe([], 480, config, now=.5)
        self.assertTrue(held.startup_ignored, 'commanded distance released a physically stationary departure')
        held.record_odometry_distance(.26)
        held.observe([], 480, config, now=.6)
        self.assertFalse(held.startup_ignored)
        self.assertEqual(held.ledger.counts['passed'], 0)
        self.assertEqual(held.ledger.counts['uncertain'], 1)
        for invalid in (0., math.nan, True, .6):
            with self.assertRaises(ValueError):
                replace(config, stop_marker_departure_hold_distance=invalid).validate()

    def test_logged_reverse_start_overshoot_is_departure_after_wait(self):
        for direction in ('down', 'up'):
            config = replace(ControlConfig(forward_ignore_start_marker=False), drive_direction='reverse',
                             image_forward_direction=direction, stop_marker_target_ratio=.46)
            sign = -1. if direction == 'down' else 1.
            gate = StopMarkerGate()
            for frame in range(50):
                gate.observe([], 480, config, now=frame*.05)
                gate.record_motion(0., .05, config)
            self.assertFalse(gate.startup_checked)
            for index, pixel_error in enumerate((-46.51, -36.76, -36.51, -46.36)):
                now = 3.+index
                gate.observe([{'x': 320., 'y': .46*480+sign*pixel_error,
                               'kind': 'intermediate'}], 480, config, now=now)
                self.assertEqual(gate.limit(-.025, config), (-.025, 'searching'))
                gate.record_motion(-.005, .1, config)
                gate.observe([], 480, config, now=now+.7)
                self.assertTrue(gate.startup_ignored)
                self.assertEqual(gate.limit(-.025, config), (-.025, 'searching'))
            self.assertEqual(gate.station_id, 0)
        for budget in (0., .051):
            with self.assertRaises(ValueError):
                replace(ControlConfig(forward_ignore_start_marker=False), reverse_start_marker_max_travel=budget).validate()

    def test_forward_start_bar_is_ignored_but_next_station_is_not(self):
        for direction in ('up', 'down'):
            config = replace(ControlConfig(), image_forward_direction=direction)
            self.assertTrue(config.forward_ignore_start_marker)
            sign = -1 if direction == 'up' else 1
            gate = StopMarkerGate()
            def observe(error, count=1):
                for _ in range(count):
                    gate.observe([{'x': 320., 'y': 240.+sign*error, 'kind': 'intermediate'}], 480, config)
            observe(8.7, 6)  # 现场日志：启动时就压在+8.7px出发横线。
            self.assertTrue(gate.startup_ignored)
            self.assertEqual(gate.limit(.075, config), (.075, 'searching'))
            for error in (-10., -35., -60., -80., -235.):
                observe(error)
            self.assertFalse(gate.startup_ignored)
            for error in (100., 75., 50., 25., 10., 0.):
                observe(error, 4)
            self.assertTrue(gate.arrived)
            self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
            gate = StopMarkerGate()
            observe(124.2, 4)  # 现场日志：启动时前方124像素，也属于出发线。
            self.assertTrue(gate.startup_ignored)
            self.assertEqual(gate.limit(.075, config), (.075, 'searching'))
            for error in (100., 75., 50., 25., 10., 0., -25., -50., -80., -235.):
                observe(error, 4)
            self.assertFalse(gate.arrived)
            self.assertFalse(gate.startup_ignored)
            for error in (120., 95., 65., 35., 10., 0.):
                observe(error, 4)
            self.assertTrue(gate.arrived)  # 后续新横线仍正常停车。
            for initial_error in (-220., 220.):
                gate = StopMarkerGate()
                observe(initial_error)
                self.assertTrue(gate.startup_checked)
                self.assertEqual(gate.limit(.075, config), (.075, 'searching'))  # 覆盖图像上、下边缘。
            gate = StopMarkerGate()
            gate.record_motion(.075, .2, config)  # 已走15mm，不能把后来遇到的横线当出发线。
            observe(8.7, 6)
            self.assertTrue(gate.arrived)
        with self.assertRaises(ValueError):
            replace(ControlConfig(), forward_ignore_start_marker='true').validate()

    def test_start_marker_policy_does_not_skip_normal_arrival(self):
        marker = {'x': 320., 'y': 240., 'kind': 'intermediate'}
        reverse = replace(ControlConfig(forward_ignore_start_marker=False), drive_direction='reverse')
        for config in (ControlConfig(forward_ignore_start_marker=False), replace(reverse, reverse_ignore_start_marker=False)):
            gate = StopMarkerGate()
            for i in range(7):
                gate.observe([marker], 480, config, now=i*.05)
            self.assertEqual(gate.limit(.025, config), (0., 'stopped_intermediate'))
        gate = StopMarkerGate()
        for i in range(reverse.stop_marker_average_frames):
            gate.observe([], 480, reverse, now=i*.05)
        gate.record_motion(-.025, .4, reverse)  # 离站识别已超过10mm指令行程。
        for i in range(7):
            gate.observe([marker], 480, reverse, now=.3+i*.05)
        self.assertEqual(gate.limit(-.025, reverse), (0., 'stopped_intermediate'))
        # 首次看到时在容差外，后续进入容差的站点不能当出发站略过。
        gate = StopMarkerGate()
        gate.observe([dict(marker, y=220.)], 480, reverse, now=0.)
        for i in range(7):
            gate.observe([marker], 480, reverse, now=.05+i*.05)
        self.assertEqual(gate.limit(-.025, reverse), (0., 'stopped_intermediate'))
        # 漏检期间已经走出限定距离才取消去重，不能原地等半秒就重新锁停。
        gate = StopMarkerGate()
        gate.observe([marker], 480, reverse, now=0.)
        gate.record_motion(-.025, reverse.stop_marker_departure_hold_distance/.025+.1, reverse)
        gate.observe([], 480, reverse, now=.6)
        self.assertFalse(gate.startup_ignored)
        for i in range(7):
            gate.observe([marker], 480, reverse, now=.7+i*.05)
        self.assertEqual(gate.limit(-.025, reverse), (0., 'stopped_intermediate'))
        with self.assertRaises(ValueError):
            replace(reverse, reverse_ignore_start_marker='true').validate()

    def test_reverse_bar_approach_sign_and_station_resume(self):
        for image_direction in ('up', 'down'):
            config = replace(ControlConfig(forward_ignore_start_marker=False), drive_direction='reverse', image_forward_direction=image_direction)
            sign = -1. if image_direction == 'down' else 1.
            gate = StopMarkerGate()
            for error in (.15, .10, .05, .02, 0.):
                for _ in range(7):
                    gate.observe([{'x': 320., 'y': 480*(.5+sign*error), 'kind': 'intermediate'}], 480, config)
                speed, state = gate.limit(-.025, config)
                if error > config.stop_marker_position_tolerance_ratio:
                    self.assertLess(speed, 0.)
                    self.assertEqual(state, 'approaching')
            self.assertEqual((speed, state), (0., 'stopped_intermediate'))
            self.assertTrue(gate.resume(config))
            gate.observe([{'x': 320., 'y': 240., 'kind': 'intermediate'}], 480, config)
            self.assertEqual(gate.limit(-.025, config), (-.025, 'searching'))

    def test_lookahead_does_not_jump_with_scan_row_phase(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        targets = []
        for phase in range(6):
            points = [(320+.3*(y-200), y) for y in range(120+phase, 470, 6)]
            geometry, reason = raw_tracking_geometry(points, 640, 480, config)
            self.assertIsNone(reason)
            targets.append((geometry['target_x'], geometry['target_y']))
            self.assertAlmostEqual(geometry['target_y'],
                (config.vehicle_reference_y_ratio+geometry['effective_lookahead_ratio'])*479)
            self.assertFalse(geometry['target_gap_interpolated'])
        for target in targets[1:]:
            self.assertAlmostEqual(target[0], targets[0][0])

    def test_bend_continuity_margin_keeps_supported_turn_and_rejects_large_gap(self):
        config = ControlConfig(forward_ignore_start_marker=False)
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
        config = ControlConfig(forward_ignore_start_marker=False)
        near = [(320, y) for y in range(120, 220, 6)]
        far = [(320, y) for y in range(320, 470, 6)]
        near[-1] = (340, near[-1][1])
        far[0] = (300, far[0][1])
        geometry, reason = raw_tracking_geometry(near + far, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(len(geometry['bridged_gaps']), 1)
        self.assertLessEqual(geometry['trusted_speed_factor'], config.bridge_speed_factor)

    def test_intersecting_extensions_join_only_inside_gap(self):
        config = ControlConfig(forward_ignore_start_marker=False)
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
        config = ControlConfig(forward_ignore_start_marker=False)
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

    def test_occluded_leading_fragments_do_not_hide_supported_path(self):
        for direction in ('down', 'up'):
            config = replace(ControlConfig(forward_ignore_start_marker=False), image_forward_direction=direction,
                             vehicle_reference_y_ratio=-.05 if direction == 'down' else 1.05)
            points = [(100, 6), (500, 54), (320, 120), (320, 126)]
            points += [(320, y) for y in range(180, 470, 6)]
            if direction == 'up':
                points = [(x, 479-y) for x, y in points]
            geometry, reason = raw_tracking_geometry(points, 640, 480, config)
            self.assertIsNone(reason)
            self.assertEqual(geometry['ignored_leading_points'], 2)
            self.assertTrue(geometry['lookahead_supported'])
            self.assertEqual(geometry['trusted_speed_factor'], 1.)
        # 偏移到另一根线或超长空白不能按碎点跳过。
        for start_x, far_start in ((100, 180), (320, 330)):
            points = [(start_x, 120), (start_x, 126)]
            points += [(320, y) for y in range(far_start, 470, 6)]
            self.assertIsNotNone(raw_tracking_geometry(points, 640, 480, ControlConfig(forward_ignore_start_marker=False))[1])

    def test_marker_dropout_creep_is_bounded_and_only_far_from_arrival(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        for speed in (.075, -.025):
            gate = StopMarkerGate()
            for i in range(7):
                gate.observe([{'x': 320., 'y': 400., 'kind': 'intermediate'}],
                             480, config, now=i*.01)
            gate.observe([], 480, config, now=.08)
            value, state = gate.limit(speed, config, now=.1)
            self.assertEqual(state, 'missing_approaching')
            self.assertAlmostEqual(value, math.copysign(config.stop_marker_creep_speed, speed))
            self.assertEqual(gate.limit(speed, config, now=.68), (0., 'missing'))
            self.assertEqual(gate.limit(speed, replace(config, stop_marker_dropout_grace_sec=0.),
                                        now=.1), (0., 'missing'))
        for y in (260., 240.):
            gate = StopMarkerGate()
            for i in range(7):
                gate.observe([{'x': 320., 'y': y, 'kind': 'intermediate'}],
                             480, config, now=i*.01)
            gate.observe([], 480, config, now=.08)
            self.assertEqual(gate.limit(.075, config, now=.1)[0], 0.)
        for grace in (-.01, .61):
            with self.assertRaises(ValueError):
                replace(config, stop_marker_dropout_grace_sec=grace).validate()

    def test_stop_bar_half_second_dropout_keeps_bounded_creep(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        gate = StopMarkerGate()
        marker = {'x': 320., 'y': 340., 'kind': 'intermediate'}
        for now in (0., .1, .2):
            gate.observe([marker], 480, config, now=now)
        for now in (.3, .4, .5, .6, .7):
            gate.observe([], 480, config, now=now)
            speed, state = gate.limit(.075, config, now=now, dt=.1)
            if now <= .6:
                self.assertEqual(state, 'missing_approaching')
                self.assertEqual(speed, config.stop_marker_creep_speed)
            else:
                self.assertEqual((speed, state), (0., 'missing'))
            gate.record_motion(speed, .1, config)
        self.assertLessEqual(gate.dropout_commanded_distance, .005)
        self.assertEqual(gate.limit(.075, config, now=.81), (0., 'missing'))
        gate.observe([marker], 480, config, now=.82)
        self.assertEqual(gate.dropout_commanded_distance, 0.)
        gate.observe([], 480, config, now=.83)
        gate.record_motion(.008, .7, config)
        self.assertEqual(gate.limit(.075, config, now=.84), (0., 'missing'))
        for value in (-.001, .006, math.inf, math.nan):
            with self.assertRaises(ValueError):
                replace(config, stop_marker_dropout_max_distance=value).validate()

    def test_short_near_segment_can_advance_without_a_far_segment(self):
        config = ControlConfig(forward_ignore_start_marker=False)
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
        config = ControlConfig(forward_ignore_start_marker=False)
        near = [(320, y) for y in range(120, 270, 6)]
        distant = [(500, y) for y in range(350, 470, 6)]
        geometry, reason = raw_tracking_geometry(near + distant, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(geometry['trusted_end_y'], near[-1][1])
        self.assertLessEqual(geometry['target_y'], near[-1][1])
        self.assertEqual(geometry['trusted_speed_factor'], 1)  # 前瞻点已在实测近段内，不因更远处断线降速。
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
        self.assertGreaterEqual(newer['trusted_speed_factor'], geometry['trusted_speed_factor'])
        # 近段不足时，即使远处还有长线，也必须停下。
        short = [(320, y) for y in range(120, 144, 6)]
        self.assertEqual(raw_tracking_geometry(short + distant, 640, 480, config)[1],
                         'insufficient_forward_span')

    def test_stop_cross_final_t_and_turn_are_distinguished(self):
        config = ControlConfig(forward_ignore_start_marker=False)
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
        config = ControlConfig(forward_ignore_start_marker=False)
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
            gate.observe([{'x': 320., 'y': 205., 'kind': 'final'}], 480, config)
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_final'))
        self.assertFalse(gate.resume(config))

    def test_station_stays_stopped_until_new_resume_and_does_not_retrigger(self):
        config = ControlConfig(forward_ignore_start_marker=False)
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
        # 显式精细配置仍受支持；现场默认采用更宽容差和三帧平均。
        config = replace(ControlConfig(forward_ignore_start_marker=False), stop_marker_position_tolerance_ratio=.008,
                         stop_marker_overshoot_tolerance_ratio=.02, stop_marker_average_frames=5)
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
        observe(4, 7)  # 已经到站，后续位置抖动不能使小车重新行走。
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        observe(-10, 7)
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        self.assertEqual(gate.station_id, 1)

    def test_stopped_bar_position_uses_fixed_axis_but_still_detects_real_motion(self):
        config = ControlConfig(forward_ignore_start_marker=False)
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
        config = ControlConfig(forward_ignore_start_marker=False)
        gate = StopMarkerGate()
        marker = {'x': 320., 'y': 240., 'kind': 'intermediate'}
        for _ in range(7):
            gate.observe([marker], 480, config)
        self.assertEqual(gate.state(config), 'stopped_intermediate')
        gate.observe([], 480, config)
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        self.assertTrue(gate.missing)
        self.assertFalse(gate.resume(config))

    def test_arrival_accepts_logged_overshoot_with_intermittent_missing_frames(self):
        # 日志中约 -12/-17/-21px 的抖动属于轻微越线；漏帧不能一直清空到位确认。
        for direction in ('down', 'up'):
            config = replace(ControlConfig(forward_ignore_start_marker=False), image_forward_direction=direction,
                             stop_marker_target_ratio=.46)
            forward = 1. if direction == 'down' else -1.
            gate = StopMarkerGate()
            for index, error in enumerate((12., -12., -17., -21.)):
                now = index*.15
                gate.observe([{'x': 320., 'y': .46*480+forward*error,
                               'kind': 'intermediate'}], 480, config, now=now)
                if index < 3:
                    gate.observe([], 480, config, now=now+.05)
                    self.assertEqual(gate.limit(.075, config), (0., 'missing'))
                    self.assertFalse(gate.resume(config))
            self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
            self.assertEqual(gate.station_id, 1)
            # 已到站则锁定；样本过期只影响当前观测，不撤销停车。
            gate.observe([], 480, config, now=2.)
            self.assertEqual(gate.state(config), 'stopped_intermediate')
            self.assertFalse(gate.resume(config))
            gate.observe([{'x': 320., 'y': .46*480, 'kind': 'intermediate'}],
                         480, config, now=2.05)
            self.assertTrue(gate.position_confirmed)

    def test_mean_position_controls_forward_and_accepts_slight_overshoot(self):
        # 显式精细配置仍受支持；现场默认采用更宽容差和三帧平均。
        config = replace(ControlConfig(forward_ignore_start_marker=False), stop_marker_position_tolerance_ratio=.008,
                         stop_marker_overshoot_tolerance_ratio=.02, stop_marker_average_frames=5)
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
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        observe(230.4)  # 负向容差边界：恰好越过图高2%。
        self.assertEqual(gate.limit(.075, config), (0., 'stopped_intermediate'))
        gate.reset()  # 到达前真正越过容差仍报告overshoot。
        observe(230)
        self.assertEqual(gate.limit(.075, config), (0., 'overshoot'))
        self.assertFalse(gate.resume(config))
        for bad in ({'stop_marker_average_frames': 2}, {'stop_marker_overshoot_tolerance_ratio': .004},
                    {'stop_marker_average_max_age_sec': 0}, {'stop_marker_overshoot_tolerance_ratio': .11}):
            with self.assertRaises(ValueError):
                replace(config, **bad).validate()

    def test_five_frame_mean_decides_stop_instead_of_single_frame(self):
        # 显式精细配置仍受支持；现场默认采用更宽容差和三帧平均。
        config = replace(ControlConfig(forward_ignore_start_marker=False), stop_marker_position_tolerance_ratio=.008,
                         stop_marker_overshoot_tolerance_ratio=.02, stop_marker_average_frames=5)
        gate = StopMarkerGate()
        values = []
        for index, error in enumerate((6, 6, -3, 6, 6)):
            values.append(error)
            gate.observe([{'x': 320., 'y': 240.+error, 'kind': 'intermediate'}], 480, config)
            if index < 2:
                self.assertGreater(gate.limit(.075, config)[0], 0.)
            else:
                self.assertEqual(gate.limit(.075, config)[0], 0.)
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
            config = replace(ControlConfig(forward_ignore_start_marker=False), image_forward_direction=direction, stop_marker_target_ratio=.46)
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
                self.assertGreaterEqual(error, -config.stop_marker_overshoot_tolerance_ratio*480)
                self.assertLessEqual(error, config.stop_marker_position_tolerance_ratio*480)
                self.assertEqual(gate.station_id, 1)

    def test_selected_stop_bar_cannot_be_replaced_by_a_later_bar(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        gate = StopMarkerGate()
        first = {'x': 320., 'y': 340., 'kind': 'intermediate'}
        later = {'x': 320., 'y': 420., 'kind': 'intermediate'}
        for now in (0., .05, .10):
            gate.observe([first], 480, config, now=now)
        gate.observe([later], 480, config, now=.72)
        self.assertEqual(gate.marker['y'], first['y'])
        self.assertEqual(gate.limit(.075, config, now=.72), (0., 'missing'))
        self.assertFalse(gate.arrived)
        # 原目标重新出现且位置连续，允许继续接近；没有误判为到站。
        gate.observe([dict(first, y=332.)], 480, config, now=.75)
        self.assertEqual(gate.marker['y'], 332.)
        self.assertGreater(gate.limit(.075, config, now=.75)[0], 0.)

    def test_fresh_stop_position_halts_before_average_catches_up(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        for direction in ('up', 'down'):
            cfg = replace(config, image_forward_direction=direction)
            sign = -1 if direction == 'up' else 1
            gate = StopMarkerGate()
            for error in (40., 30., 20.):
                gate.observe([{'x': 320., 'y': 240.+sign*error, 'kind': 'intermediate'}], 480, cfg)
            gate.observe([{'x': 320., 'y': 240.+sign*10., 'kind': 'intermediate'}], 480, cfg)
            self.assertGreater(gate.error, cfg.stop_marker_position_tolerance_ratio)
            self.assertEqual(gate.limit(.075, cfg), (0., 'settling'))
            self.assertFalse(gate.arrived)
            for _ in range(5):
                gate.observe([{'x': 320., 'y': 240.+sign*10., 'kind': 'intermediate'}], 480, cfg)
            self.assertEqual(gate.limit(.075, cfg), (0., 'stopped_intermediate'))
            self.assertTrue(gate.arrived)

    def test_stop_confirmation_does_not_toggle_on_one_outside_frame(self):
        cfg = ControlConfig(forward_ignore_start_marker=False)
        gate = StopMarkerGate()
        def observe(error):
            gate.observe([{'x': 320., 'y': 240.+error, 'kind': 'intermediate'}], 480, cfg)
        for error in (40., 30., 20., 10.):
            observe(error)
        self.assertEqual(gate.limit(.075, cfg), (0., 'settling'))
        observe(25.)
        self.assertEqual(gate.limit(.075, cfg), (0., 'settling'))
        observe(25.)
        self.assertEqual(gate.limit(.075, cfg), (0., 'settling'))
        observe(25.)
        self.assertGreater(gate.limit(.075, cfg)[0], 0.)
        self.assertFalse(gate.arrived)

    def test_average_discards_old_samples_and_missing_frames_do_not_count_as_zero(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        gate = StopMarkerGate()
        marker = {'x': 320., 'y': 270., 'kind': 'unknown'}
        for _ in range(5):
            gate.observe([marker], 480, config, now=0.)
        gate.observe([], 480, config, now=.1)
        self.assertEqual(gate.limit(.075, config), (0., 'missing'))
        self.assertAlmostEqual(gate.error*480, 30.)
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
        for error, heading in ((0.02, 0.02), (-0.02, -0.02), (0.03, 0.04), (-0.03, -0.04)):
            self.assertEqual(line_command(observation(
                error_normalized=error, heading_error_rad=heading), self.config)[1], 0.0)
        # A just-visible error produces a tiny correction, not a step at the threshold.
        w = line_command(observation(error_normalized=self.config.error_deadband + 0.001), self.config)[1]
        self.assertAlmostEqual(abs(w), 0.00065)
        self.assertGreater(abs(line_command(observation(error_normalized=0.4), self.config)[1]), 0.1)

    def test_curve_geometry_does_not_confuse_straight_offset_with_bend(self):
        config = ControlConfig(forward_ignore_start_marker=False)
        for points in ([(400, y) for y in range(120, 270, 6)],
                       [(240, y) for y in range(120, 270, 6)]):
            geometry, reason = raw_tracking_geometry(points, 640, 480, config)
            self.assertIsNone(reason)
            self.assertAlmostEqual(geometry['curve_strength'], 0.0)
        tilted, _ = raw_tracking_geometry(
            [(319.5 + 0.5 * (y - 120), y) for y in range(120, 270, 6)], 640, 480, config)
        self.assertAlmostEqual(tilted['curve_strength'], 0.)
        self.assertAlmostEqual(tilted['near_heading_rad'], math.atan(.5))
        # 斜直线保留朝向修正，但不能进入大横向死区、压低回中力度。
        centered = line_command(observation(**{k: tilted[k] for k in (
            'error_normalized', 'heading_error_rad', 'near_heading_rad',
            'path_heading_rad', 'curve_strength')}), config)
        self.assertEqual(centered[2], 'tracking')
        self.assertGreater(centered[1], 0.)
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
        wide_offset = line_command(dict(status, error_normalized=-0.25), config)
        self.assertEqual(centered, wide_offset)  # 放宽弯道余量，不牺牲正常前进速度。
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
        self.assertEqual(geometry['near_x'], 255)  # 轮子遮挡区不参与近端选点。
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
        self.assertEqual(tracking_geometry(shallow, 640, 480, self.config)[1], 'no_forward_line')
        shallow = [(150, 330), (300, 340), (450, 350)]
        self.assertEqual(tracking_geometry(shallow, 640, 480, self.config)[1], 'insufficient_forward_span')

    def test_motion_arrow_matches_forward_and_turn(self):
        down = ControlConfig(forward_ignore_start_marker=False)
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
        config = ControlConfig(forward_ignore_start_marker=False)
        visible = [(450, 144), (380, 240), (280, 340), (160, 459)]
        geometry, reason = tracking_geometry(visible, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(geometry['near_y'], 144)
        distant = [(430, 330), (380, 350), (280, 400), (160, 459)]
        self.assertEqual(tracking_geometry(distant, 640, 480, config)[1], 'near_line_missing')

    def test_near_ground_start_excludes_wheel_region(self):
        config = ControlConfig(forward_ignore_start_marker=False)
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


class DockDepartureTests(unittest.TestCase):
    def test_clear_requires_three_distinct_frames(self):
        from dock_departure import DockDeparture
        gate = DockDeparture()
        gate.advance(0., (0., 0.), True)
        gate.observe(False, 1)
        gate.observe(False, 1)
        self.assertEqual(gate.clear_frames, 1)
        gate.observe(True, 2)
        gate.observe(False, 3)
        gate.observe(False, 4)
        self.assertEqual(gate.state, 'active')
        gate.observe(False, 5)
        self.assertEqual(gate.state, 'complete')
        gate.advance(10., (.1, 0.), True)
        self.assertEqual(gate.state, 'complete')

    def test_distance_counts_actual_path_and_latches(self):
        from dock_departure import DockDeparture
        gate = DockDeparture()
        gate.advance(0., (0., 0.), True)
        gate.advance(1., (.16, 0.), True)
        gate.advance(2., (0., 0.), True)
        self.assertEqual(gate.fault, 'dock_departure_distance_exceeded')
        for stamp in range(3):
            gate.observe(False, stamp)
        self.assertIsNotNone(gate.fault)

    def test_timeout_includes_pauses(self):
        from dock_departure import DockDeparture
        gate = DockDeparture()
        gate.advance(0., (0., 0.), True)
        gate.advance(25., (0., 0.), True)
        self.assertEqual(gate.fault, 'dock_departure_timeout')

    def test_ordinary_start_does_not_arm(self):
        from dock_departure import DockDeparture
        gate = DockDeparture()
        gate.advance(0., (0., 0.), False)
        self.assertEqual(gate.state, 'idle')


if __name__ == '__main__':
    unittest.main()
