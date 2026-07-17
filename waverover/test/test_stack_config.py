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
import yaml


def write_identity(tmp_path, content):
    path = tmp_path / 'identity.yaml'
    path.write_text(content, encoding='utf-8')
    return path


def test_shared_defaults_and_robot_derivation():
    config = load_stack_config(require_identity=False)

    assert 'robot_name' not in config
    assert config['control_mode'] == 'fixed_wing'
    assert config['pose_source'] == 'MCS'
    assert robot_namespace(config, '30') == 'waverover_30'
    assert robot_frame(config, 'map', '30') == 'waverover_30/map'
    assert robot_frame(config, 'base', '30') == (
        'waverover_30/base_footprint'
    )
    assert robot_topic(config, 'cmd_vel', '30') == (
        '/waverover_30/cmd_vel'
    )
    assert robot_topic(config, 'waypoints', '30') == (
        '/waverover_30/waypoints'
    )
    assert robot_topic(config, 'end_trial', '30') == (
        '/waverover_30/end_trial'
    )
    assert config['waypoint_ui']['refresh_rate_hz'] == pytest.approx(1.0)
    assert robot_topic(config, 'tf', '30') == '/tf'
    assert mcs_pose_topic(config, '131') == (
        '/macortex_bridge/waverover_131/pose'
    )
    assert waypoint_global_frame(config, 'SLAM', '30') == (
        'waverover_30/map'
    )
    assert waypoint_global_frame(config, 'MCS', '30') == 'robotics_lab'


def test_shared_defaults_are_independent():
    first = load_stack_config(require_identity=False)
    first['topics']['cmd_vel'] = 'changed'
    second = load_stack_config(require_identity=False)
    assert second['topics']['cmd_vel'] == 'cmd_vel'


@pytest.mark.parametrize('value', [0, -1, float('inf'), 'fast'])
def test_invalid_waypoint_refresh_rate_is_rejected(tmp_path, value):
    defaults = load_stack_config(require_identity=False)
    defaults['waypoint_ui']['refresh_rate_hz'] = value
    config_path = tmp_path / 'defaults.yaml'
    config_path.write_text(yaml.safe_dump(defaults), encoding='utf-8')

    with pytest.raises(StackConfigError, match='refresh_rate_hz'):
        load_stack_config(require_identity=False, config_path=config_path)


def test_end_trial_topic_is_required_and_relative(tmp_path):
    defaults = load_stack_config(require_identity=False)
    defaults['topics']['end_trial'] = '/global/end_trial'
    config_path = tmp_path / 'defaults.yaml'
    config_path.write_text(yaml.safe_dump(defaults), encoding='utf-8')
    with pytest.raises(StackConfigError, match='topics.end_trial'):
        load_stack_config(require_identity=False, config_path=config_path)

    del defaults['topics']['end_trial']
    config_path.write_text(yaml.safe_dump(defaults), encoding='utf-8')
    with pytest.raises(StackConfigError, match='topics.end_trial'):
        load_stack_config(require_identity=False, config_path=config_path)


def test_explicit_identity_and_whitespace_normalization(tmp_path, monkeypatch):
    monkeypatch.delenv('WAVEROVER_IDENTITY_FILE', raising=False)
    identity = write_identity(tmp_path, 'robot_name: " 131 "\n')
    config = load_stack_config(identity_path=identity)
    assert config['robot_name'] == '131'


def test_environment_identity_and_priority(tmp_path, monkeypatch):
    explicit = write_identity(tmp_path, 'robot_name: "wrong"\n')
    environment = tmp_path / 'environment.yaml'
    environment.write_text('robot_name: "131"\n', encoding='utf-8')
    monkeypatch.setenv('WAVEROVER_IDENTITY_FILE', str(environment))
    config = load_stack_config(identity_path=explicit)
    assert config['robot_name'] == '131'


def test_missing_identity_is_only_an_error_when_required(tmp_path, monkeypatch):
    missing = tmp_path / 'missing.yaml'
    monkeypatch.delenv('WAVEROVER_IDENTITY_FILE', raising=False)
    assert 'robot_name' not in load_stack_config(require_identity=False)
    with pytest.raises(StackConfigError, match=str(missing)):
        load_stack_config(identity_path=missing)


@pytest.mark.parametrize(
    ('content', 'message'),
    [
        ('', 'mapping'),
        ('[131]\n', 'mapping'),
        ('robot_name: [broken\n', 'parse'),
        ('other: 131\n', 'missing robot_name'),
        ('robot_name: 131\nother: value\n', 'unexpected keys'),
        ('robot_name: "bad-name"\n', 'robot_name'),
        ('robot_name: ""\n', 'robot_name'),
    ],
)
def test_invalid_identities_are_actionable(
    tmp_path,
    monkeypatch,
    content,
    message,
):
    monkeypatch.delenv('WAVEROVER_IDENTITY_FILE', raising=False)
    identity = write_identity(tmp_path, content)
    with pytest.raises(StackConfigError, match=message):
        load_stack_config(identity_path=identity)


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
    config = load_stack_config(require_identity=False)
    command = manual_lr_command(config, '30')
    assert '__ns:=/waverover_30' in command
    assert 'topic:=manual_lr' in command
