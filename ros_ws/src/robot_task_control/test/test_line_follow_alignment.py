#!/usr/bin/env python3
"""Pure state-machine checks for startup alignment status handling."""

import unittest
import importlib.util
from pathlib import Path
from time import monotonic
from types import SimpleNamespace
from unittest.mock import Mock

from robot_task_control.workflow_controller import WorkflowController


class StartAlignmentWorkflowTests(unittest.TestCase):
    @staticmethod
    def workflow_state(status):
        now = monotonic()
        return SimpleNamespace(
            use_velocity_mux=True,
            phase='follow_tracking',
            follow_last_tick=now-.1,
            follow_status_time=now,
            follow_status=status,
            follow_tracking_seconds=3.0,
            follow_last_tracking=now-.5,
            follow_boundary_since=now-.2,
            follow_station_confirm_since=now-.2,
            fail_workflow=Mock(),
        )

    def test_alignment_does_not_consume_line_progress_timeout(self):
        for status in ('start_alignment_settling', 'start_alignment_waiting_for_stable_image',
                       'start_alignment_moving_forward', 'start_alignment_moving_reverse',
                       'start_alignment_moving_yaw', 'start_alignment_moving_lateral',
                       'start_alignment_departing_dock', 'start_alignment_settling_after_departure',
                       'start_alignment_waiting_for_line', 'start_alignment_waiting_for_stop'):
            with self.subTest(status=status):
                node = self.workflow_state(status)
                previous = node.follow_last_tracking
                WorkflowController.advance_line_follow(node, {'seconds': 1.0})
                self.assertEqual(node.follow_tracking_seconds, 3.0)
                self.assertGreater(node.follow_last_tracking, previous)
                self.assertIsNone(node.follow_boundary_since)
                self.assertIsNone(node.follow_station_confirm_since)
                node.fail_workflow.assert_not_called()

    def test_alignment_failure_stops_workflow(self):
        node = self.workflow_state('start_alignment_failed_ground_mapping_invalid')
        WorkflowController.advance_line_follow(node, {'seconds': 1.0})
        node.fail_workflow.assert_called_once()
        self.assertIn('起步横线对齐失败', node.fail_workflow.call_args.args[0])

    def test_standard_route_failure_stops_without_advancing_workflow(self):
        for status in ('standard_visual_loss_timeout', 'standard_visual_loss_distance_limit',
                       'standard_route_deviation', 'standard_finish_search_limit'):
            node = self.workflow_state(status)
            WorkflowController.advance_line_follow(node, {'seconds': 1.0})
            node.fail_workflow.assert_called_once()
            self.assertIn(status, node.fail_workflow.call_args.args[0])
            self.assertEqual(node.follow_tracking_seconds, 3.0)


class LineFollowSpeedLaunchTests(unittest.TestCase):
    def test_recording_speed_is_independent_of_normal_cruise(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

        path = Path(__file__).resolve().parents[1]/'launch'/'line_follow_only.launch.py'
        spec = importlib.util.spec_from_file_location('line_speed_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        entities = module.generate_launch_description().entities
        include = next(item for item in entities if isinstance(item, IncludeLaunchDescription))
        arguments = dict(include.launch_arguments)
        for mode, overrides, expected in (
                ('teach', {}, .03), ('vision', {}, .1), ('replay', {}, .1),
                ('teach', {'line_follow_speed': '0.12'}, .03),
                ('teach', {'line_follow_speed': '0.12', 'line_record_speed': '0.025'}, .025),
                ('teach', {'line_follow_speed': '0.12', 'start_alignment_min_lateral_speed': '0.025'}, .03),
                ('vision', {'line_record_speed': '0.02'}, .1)):
            with self.subTest(mode=mode, overrides=overrides):
                context = LaunchContext()
                context.launch_configurations.update(navigation_mode=mode, **overrides)
                for item in entities:
                    if isinstance(item, DeclareLaunchArgument) and item.name in (
                            'line_follow_speed', 'line_record_speed', 'line_reverse_speed',
                            'start_alignment_min_lateral_speed', 'start_alignment_lateral_speed'):
                        item.execute(context)
                def resolved(name):
                    return float(perform_substitutions(context,
                        normalize_to_list_of_substitutions(arguments[name])))
                self.assertAlmostEqual(resolved('line_follow_speed'), expected)
                self.assertAlmostEqual(resolved('line_reverse_speed'), .05)
                self.assertAlmostEqual(resolved('line_start_alignment_min_lateral_speed'),
                                       float(overrides.get('start_alignment_min_lateral_speed', '.02')))
                self.assertAlmostEqual(resolved('line_start_alignment_lateral_speed'), .03)


if __name__ == '__main__':
    unittest.main()
