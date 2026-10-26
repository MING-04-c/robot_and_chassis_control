#!/usr/bin/env python3
"""Evaluate launch with fake Nodes: no hardware, GUI or robot commands."""

from pathlib import Path

import launch_ros.actions
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.utilities import evaluate_parameters, normalize_parameters


seen = []
original = launch_ros.actions.Node


def fake_node(**kwargs):
    def inspect(context):
        params = evaluate_parameters(context, normalize_parameters(kwargs.get('parameters', [])))
        overrides = {}
        for item in params:
            if isinstance(item, dict):
                overrides.update(item)
        seen.append((kwargs.get('name'), overrides))
        return []
    return OpaqueFunction(function=inspect, condition=kwargs.get('condition'))


try:
    launch_ros.actions.Node = fake_node
    source = Path(__file__).resolve().parents[1] / 'launch' / 'line_follow.launch.py'
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(source)),
            launch_arguments={'show_window': 'true', 'dry_run': 'true',
                              'enabled': 'false'}.items(),
        ),
    ]))
    assert service.run() == 0
    controllers = [params for name, params in seen if name == 'line_follow_controller']
    trackers = [params for name, params in seen if name == 'line_tracker']
    assert len(controllers) == 1 and controllers[0]['show_window'] is True, seen
    assert controllers[0]['enabled'] is False, seen
    assert len([p for name, p in seen if name == 'line_exposure_controller']) == 1, seen
    assert len(trackers) == 1 and trackers[0]['show_window'] is False, seen
    print('PASS: camera preview disabled locally; controller preview remains enabled')
finally:
    launch_ros.actions.Node = original
