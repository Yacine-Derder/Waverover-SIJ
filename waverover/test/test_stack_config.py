import pytest

from waverover.namespaced_manual_lr import manual_lr_command
from waverover.stack_config import (
    load_stack_config,
    mcs_pose_topic,
    normalize_control_mode,
    normalize_pose_source,
    robot_frame,
    robot_namespace,
    robot_topic,
    StackConfigError,
    waypoint_global_frame,
)


def test_canonical_defaults_and_robot_derivation():
    config = load_stack_config()

    assert isinstance(config['robot_name'], str)
    assert config['robot_name']
    assert config['control_mode'] == 'fixed_wing'
    assert config['pose_source'] == 'SLAM'
    assert robot_namespace(config) == (
        config['namespace_prefix'] + config['robot_name']
    )
    assert robot_namespace(config, '30') == 'robot_30'
    assert robot_frame(config, 'map', '30') == 'robot_30/map'
    assert robot_topic(config, 'cmd_vel', '30') == '/robot_30/cmd_vel'
    assert robot_topic(config, 'tf', '30') == '/tf'
    assert mcs_pose_topic(config, '29') == (
        '/macortex_bridge/robot_29/pose'
    )
    assert mcs_pose_topic(config, '30') == (
        '/macortex_bridge/robot_30/pose'
    )
    assert waypoint_global_frame(config, 'SLAM', '30') == 'robot_30/map'
    assert waypoint_global_frame(config, 'MCS', '30') == 'robotics_lab'


@pytest.mark.parametrize('value', ['twist', 'FIXED_WING', 'manual_lr'])
def test_control_modes_are_normalized(value):
    assert normalize_control_mode(value) in (
        'twist',
        'fixed_wing',
        'manual_lr',
    )


@pytest.mark.parametrize('value', ['SLAM', 'slam', 'MCS', 'mcs'])
def test_pose_sources_are_normalized(value):
    assert normalize_pose_source(value) in ('SLAM', 'MCS')


def test_unknown_pose_source_is_rejected():
    with pytest.raises(StackConfigError, match='pose_source'):
        normalize_pose_source('odometry')


def test_manual_ui_command_uses_derived_namespace():
    command = manual_lr_command(load_stack_config(), '30')

    assert '__ns:=/robot_30' in command
    assert 'topic:=manual_lr' in command
