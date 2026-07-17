import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

from waverover.stack_config import load_stack_config
from waverover_controller.waypoint_controller import ControllerConfig


@pytest.fixture(autouse=True)
def test_identity(tmp_path, monkeypatch):
    identity = tmp_path / 'identity.yaml'
    identity.write_text('robot_name: "test"\n', encoding='utf-8')
    monkeypatch.setenv('WAVEROVER_IDENTITY_FILE', str(identity))


def _load_waypoint_launch_module():
    launch_path = (
        Path(__file__).parents[1]
        / 'launch'
        / 'waypoint_controller.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'waypoint_controller_launch',
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_controller_defaults_are_central_and_robot_derived():
    stack_config = load_stack_config(require_identity=False)
    config = ControllerConfig.from_stack_defaults(stack_config, '30')

    assert config.control_mode == 'fixed_wing'
    assert config.pose_source == 'MCS'
    assert config.global_frame == 'waverover_30/map'
    assert config.robot_frame == 'waverover_30/base_footprint'
    assert config.cmd_vel_topic == 'cmd_vel'
    assert config.waypoint_topic == 'waypoints'
    assert config.end_trial_topic == 'end_trial'


def test_mcs_launch_selection_uses_external_pose_and_lab_frame():
    module = _load_waypoint_launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_name': '29',
        'control_mode': 'fixed_wing',
        'pose_source': 'MCS',
        'params_file': '',
        'mcs_pose_topic': '',
        'mcs_frame': 'robotics_lab',
        'mcs_pose_timeout_sec': '0.5',
    })

    node = next(
        action
        for action in module._launch_waypoint_controller(context)
        if isinstance(action, Node)
    )
    parameters = evaluate_parameters(context, node._Node__parameters)[0]

    assert parameters['pose_source'] == 'MCS'
    assert parameters['global_frame'] == 'robotics_lab'
    assert parameters['mcs_pose_topic'] == (
        '/macortex_bridge/waverover_29/pose'
    )
    assert parameters['mcs_pose_timeout_sec'] == pytest.approx(0.5)
    assert parameters['end_trial_topic'] == 'end_trial'


def test_slam_launch_selection_keeps_namespaced_tf_frames():
    module = _load_waypoint_launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_name': '30',
        'control_mode': 'twist',
        'pose_source': 'SLAM',
        'params_file': '',
        'mcs_pose_topic': '',
        'mcs_frame': 'robotics_lab',
        'mcs_pose_timeout_sec': '0.5',
    })

    node = module._launch_waypoint_controller(context)[0]
    parameters = evaluate_parameters(context, node._Node__parameters)[0]

    assert parameters['pose_source'] == 'SLAM'
    assert parameters['global_frame'] == 'waverover_30/map'
    assert parameters['robot_frame'] == 'waverover_30/base_footprint'
    assert parameters['mcs_pose_topic'] == (
        '/macortex_bridge/waverover_30/pose'
    )
