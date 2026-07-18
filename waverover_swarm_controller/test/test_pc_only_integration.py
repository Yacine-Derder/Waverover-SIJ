import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from rclpy.validate_topic_name import validate_topic_name

from waverover.stack_config import load_stack_config
from waverover_swarm_controller.coordinator_node import (
    SwarmCoordinator,
    predicted_path_topic,
)


def package_root():
    return Path(__file__).parents[1]


def test_coordinator_uses_existing_interfaces_and_never_cmd_vel():
    source = (
        package_root()
        / 'waverover_swarm_controller'
        / 'coordinator_node.py'
    ).read_text(encoding='utf-8')

    assert 'load_stack_config(require_identity=False)' in source
    assert "robot_topic(stack_config, 'waypoints'" in source
    assert "robot_topic(stack_config, 'end_trial'" in source
    assert 'mcs_pose_topic(stack_config' in source
    assert 'cmd_vel' not in source


def test_numeric_rover_id_has_valid_namespaced_predicted_path_topic():
    stack_config = load_stack_config(require_identity=False)

    relative = predicted_path_topic(stack_config, '134')
    resolved = '/waverover_swarm/' + relative

    validate_topic_name(resolved)
    assert resolved == '/waverover_swarm/predicted_path/waverover_134'


def test_transient_pose_warning_clears_after_valid_cycle_but_fault_stays_latched():
    class FakeCoordinator:
        def __init__(self, faulted):
            self.latest_stop_reason = 'Missing poses for: 134.'
            self.dispatcher = SimpleNamespace(
                faulted=faulted,
                update_pending=lambda _setpoints: None,
            )
            self.armed = False
            self.latest_snapshot = None
            self.latest_result = None
            self._snapshot = lambda: object()
            self._compute_valid_result = lambda _snapshot: SimpleNamespace(
                setpoints={'134': (0.0, 0.0)}
            )
            self._publish_visualization = lambda _snapshot, _result: None
            self._publish_diagnostics = lambda: None

    recovered = FakeCoordinator(faulted=False)
    SwarmCoordinator._control_cycle(recovered)
    assert recovered.latest_stop_reason == ''

    faulted = FakeCoordinator(faulted=True)
    SwarmCoordinator._control_cycle(faulted)
    assert faulted.latest_stop_reason == 'Missing poses for: 134.'


def test_dry_run_arm_request_is_rejected():
    coordinator = SimpleNamespace(
        armed=False,
        config=SimpleNamespace(safety=SimpleNamespace(dry_run=True)),
    )
    request = SimpleNamespace(data=True)
    response = SimpleNamespace(success=None, message='')

    returned = SwarmCoordinator._arm_callback(coordinator, request, response)

    assert returned is response
    assert not response.success
    assert response.message == 'Cannot arm while dry_run is true.'


def test_launch_is_standalone_dry_run_by_default():
    launch_path = package_root() / 'launch' / 'swarm_controller.launch.py'
    spec = importlib.util.spec_from_file_location('swarm_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()

    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    assert arguments['dry_run'].default_value[0].text == 'true'
    assert arguments['algorithm'].default_value[0].text == 'heuristic'
    assert len(nodes) == 1
    assert nodes[0]._Node__package == 'waverover_swarm_controller'


def test_synthetic_launch_exposes_typed_arguments_under_swarm_namespace():
    launch_path = package_root() / 'launch' / 'synthetic_mcs.launch.py'
    spec = importlib.util.spec_from_file_location('synthetic_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()

    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    assert set(arguments) == {
        'config_file', 'rate_hz', 'radius_m', 'angle_offset_rad', 'yaw_rad'
    }
    assert len(nodes) == 1
    assert nodes[0].node_executable == 'synthetic_mcs'
    assert nodes[0]._Node__node_namespace == 'waverover_swarm'
