"""Prevent static YAML from overriding launch-selected runtime wiring."""

import re
from pathlib import Path


def test_ekf_tuning_does_not_pin_launch_selected_topics_or_frames():
    config = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'zed_vectornav_ekf.yaml'
    ).read_text(encoding='utf-8')
    launch_selected = (
        'odom0',
        'imu0',
        'map_frame',
        'odom_frame',
        'base_link_frame',
        'world_frame',
        'publish_tf',
        'use_sim_time',
    )
    for parameter in launch_selected:
        assert re.search(
            rf'^\s*{parameter}:', config, flags=re.MULTILINE
        ) is None, (
            f'{parameter} is selected by zed_vectornav_ekf.launch.py; '
            'a node-specific YAML value overrides launch arguments on Foxy'
        )


def test_control_tuning_does_not_pin_selected_command_source():
    config = (
        Path(__file__).resolve().parents[2]
        / 'tardigrade_control'
        / 'config'
        / 'control.yaml'
    ).read_text(encoding='utf-8')
    assert re.search(
        r'^\s*active_source:', config, flags=re.MULTILINE
    ) is None
