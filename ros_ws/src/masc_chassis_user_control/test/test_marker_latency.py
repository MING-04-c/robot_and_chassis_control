#!/usr/bin/env python3
"""Check the real detector adapter without starting nodes or hardware."""
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from cv_bridge import CvBridge

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from line_follow_node import LineFollowNode
from line_follow_control import ControlConfig


class MarkerLatencyTests(unittest.TestCase):
    def test_marker_pending_requires_fresh_matching_mainline_and_no_other_block(self):
        node = SimpleNamespace(
            frame_sync=True, path_valid=True, input_timeout=.5,
            path_header=SimpleNamespace(stamp=SimpleNamespace(sec=9, nanosec=800_000_000)),
            status_time=9.8, path_time=9.8,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
        for reason in ('stop_marker_mask_timeout', 'stop_marker_mask_stale'):
            self.assertTrue(LineFollowNode.alignment_marker_pending(node, 10., reason))
        for reason in ('camera_timeout', 'stale_image_stamp', 'chassis_not_ready',
                       'odom_stale', 'communication_timeout', None):
            self.assertFalse(LineFollowNode.alignment_marker_pending(node, 10., reason))
        node.path_header.stamp.sec = 8
        self.assertFalse(LineFollowNode.alignment_marker_pending(node, 10., 'stop_marker_mask_stale'))
        node.path_header.stamp.sec = 9
        node.path_valid = False
        self.assertFalse(LineFollowNode.alignment_marker_pending(node, 10., 'stop_marker_mask_stale'))

    def test_stale_source_is_not_submitted_and_fresh_source_keeps_capture_stamp(self):
        mask = CvBridge().cv2_to_imgmsg(np.zeros((48, 64), np.uint8), encoding='mono8')
        mask.header.stamp.sec = 10
        requests = []
        node = SimpleNamespace(
            config=ControlConfig(), marker_stamp=None, input_timeout=.5,
            status={'image_height': 48, 'image_width': 64, 'processing_ms': 12.},
            path_points=[], marker_gate=SimpleNamespace(reference_marker=None),
            marker_worker=SimpleNamespace(submit=requests.append),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_600_000_000)))
        LineFollowNode.accept_marker_mask(node, mask)
        self.assertFalse(requests)
        mask.header.stamp.nanosec = 400_000_000
        LineFollowNode.accept_marker_mask(node, mask)
        self.assertEqual(len(requests), 1)
        with patch('line_follow_node.time.monotonic', side_effect=[20., 20.02, 20.02]):
            request = list(requests[0])
            request[5] = 19.95
            header, config, _, diagnostics, height = LineFollowNode.detect_marker_frame(request)
        self.assertIs(header, mask.header)
        self.assertIs(config, node.config)
        self.assertEqual(height, 48)
        self.assertEqual(diagnostics['upstream_age_ms'], 200.)
        self.assertEqual(diagnostics['tracker_processing_ms'], 12.)
        self.assertEqual(diagnostics['worker_wait_ms'], 50.)
        self.assertEqual(diagnostics['detector_processing_ms'], 20.)

    def test_expired_result_does_not_refresh_validity(self):
        mask = CvBridge().cv2_to_imgmsg(np.zeros((48, 64), np.uint8), encoding='mono8')
        mask.header.stamp.sec = 10
        config = ControlConfig()
        diagnostics = dict(worker_finished_at=time.monotonic(), upstream_age_ms=200.,
                           worker_wait_ms=200., detector_processing_ms=200.)
        warnings = []
        node = SimpleNamespace(
            marker_worker=SimpleNamespace(take=lambda: (1, (mask.header, config, [], diagnostics, 48), None)),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_700_000_000)),
            config=config, marker_header=None, marker_time=None, input_timeout=.5,
            last_marker_latency_log=float('-inf'),
            get_logger=lambda: SimpleNamespace(warning=warnings.append))
        LineFollowNode.collect_marker_result(node)
        self.assertIsNone(node.marker_header)
        self.assertIsNone(node.marker_time)
        self.assertEqual(len(warnings), 1)
        self.assertIn('700ms', warnings[0])
        self.assertIn('后台等待=200ms', warnings[0])


if __name__ == '__main__':
    unittest.main()
