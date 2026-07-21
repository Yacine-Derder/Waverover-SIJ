from dataclasses import replace
import importlib.util
from pathlib import Path
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from rclpy.validate_topic_name import validate_topic_name

from waverover.stack_config import load_stack_config
import waverover_swarm_controller.coordinator_node as coordinator_module
from waverover_swarm_controller.coordinator_node import (
    SwarmCoordinator,
    predicted_path_topic,
)
from waverover_swarm_controller.models import ControllerResult
from waverover_swarm_controller.waypoint_dispatcher import WaypointDispatcher


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
                commanded_robot_ids=(),
                update_pending=lambda _setpoints: None,
            )
            self.latest_snapshot = None
            self.latest_result = None
            self._snapshot = lambda: object()
            self._compute_valid_result = lambda _snapshot: SimpleNamespace(
                setpoints={'134': (0.0, 0.0)}
            )
            self._publish_visualization = lambda _snapshot, _result: None
            self._publish_controller_telemetry = lambda *_args: None
            self._publish_diagnostics = lambda: None
            self._dispatch = lambda _snapshot: None

    recovered = FakeCoordinator(faulted=False)
    SwarmCoordinator._control_cycle(recovered)
    assert recovered.latest_stop_reason == ''

    faulted = FakeCoordinator(faulted=True)
    SwarmCoordinator._control_cycle(faulted)
    assert faulted.latest_stop_reason == 'Missing poses for: 134.'


def test_coordinator_has_no_arm_interface_or_state():
    source = (
        package_root() / 'waverover_swarm_controller' / 'coordinator_node.py'
    ).read_text(encoding='utf-8')

    assert 'std_srvs' not in source
    assert 'arm_service' not in source
    assert 'self.armed' not in source


def test_dispatch_suppresses_dry_run_and_live_mode_publishes_immediately(
    example_config, snapshot, monkeypatch
):
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    def coordinator(dry_run):
        config = replace(
            example_config,
            safety=replace(example_config.safety, dry_run=dry_run),
        )
        dispatcher = WaypointDispatcher(
            snapshot.robots, config.waypoint_dispatch
        )
        dispatcher.update_pending({
            robot_id: state.position
            for robot_id, state in snapshot.robots.items()
        })
        publishers = {robot_id: Publisher() for robot_id in snapshot.robots}
        return SimpleNamespace(
            config=config,
            dispatcher=dispatcher,
            waypoint_publishers=publishers,
            latest_handoff='none',
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: Time())
            ),
            get_logger=lambda: SimpleNamespace(info=lambda _message: None),
            _stop_trial=lambda _reason: None,
        ), publishers

    now = [10.0]
    monkeypatch.setattr(coordinator_module.time, 'monotonic', lambda: now[0])

    dry, dry_publishers = coordinator(True)
    SwarmCoordinator._dispatch(dry, snapshot)
    assert all(not publisher.messages for publisher in dry_publishers.values())
    assert dry.dispatcher.commanded_robot_ids == ()

    live, live_publishers = coordinator(False)
    SwarmCoordinator._dispatch(live, snapshot)
    assert all(len(publisher.messages) == 1
               for publisher in live_publishers.values())
    assert live.dispatcher.commanded_robot_ids == tuple(sorted(snapshot.robots))

    now[0] = 10.9
    SwarmCoordinator._dispatch(live, snapshot)
    assert all(len(publisher.messages) == 1
               for publisher in live_publishers.values())

    now[0] = 11.0
    SwarmCoordinator._dispatch(live, snapshot)
    assert all(len(publisher.messages) == 2
               for publisher in live_publishers.values())
    for publisher in live_publishers.values():
        first, refreshed = publisher.messages
        assert (first.point.x, first.point.y) == (
            refreshed.point.x, refreshed.point.y
        )


def test_dry_run_stop_never_publishes_end_trial(example_config, snapshot):
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    dispatcher = WaypointDispatcher(
        snapshot.robots, example_config.waypoint_dispatch
    )
    for state in dispatcher.states.values():
        state.ever_commanded = True
    publishers = {robot_id: Publisher() for robot_id in snapshot.robots}
    coordinator = SimpleNamespace(
        config=example_config,
        dispatcher=dispatcher,
        end_trial_publishers=publishers,
        latest_stop_reason='',
        _end_sent_for_stop=False,
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )

    SwarmCoordinator._stop_trial(coordinator, 'test stop')

    assert all(not publisher.messages for publisher in publishers.values())
    assert dispatcher.faulted


def test_live_cleanup_faults_and_sends_one_end_trial_per_commanded_rover(
    example_config, snapshot
):
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    config = replace(
        example_config,
        safety=replace(example_config.safety, dry_run=False),
    )
    dispatcher = WaypointDispatcher(snapshot.robots, config.waypoint_dispatch)
    for state in dispatcher.states.values():
        state.ever_commanded = True
    publishers = {robot_id: Publisher() for robot_id in snapshot.robots}
    coordinator = SimpleNamespace(
        config=config,
        dispatcher=dispatcher,
        end_trial_publishers=publishers,
        latest_stop_reason='',
        _end_sent_for_stop=False,
        _cleanup_complete=False,
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )
    coordinator._stop_trial = lambda reason: SwarmCoordinator._stop_trial(
        coordinator, reason
    )

    commanded = SwarmCoordinator.cleanup(coordinator)
    second = SwarmCoordinator.cleanup(coordinator)

    assert commanded == tuple(sorted(snapshot.robots))
    assert second == ()
    assert dispatcher.faulted
    assert all(len(publisher.messages) == 1
               for publisher in publishers.values())


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


def test_safety_rejected_optimization_never_reaches_dispatch_and_recovers(
    example_config, snapshot
):
    now = time.monotonic()
    unsafe_points = {
        robot_id: state.position
        for robot_id, state in snapshot.robots.items()
    }
    unsafe_points['robot_10'] = (1.0, 1.0)
    unsafe_points['robot_2'] = (1.01, 1.0)
    rejected = ControllerResult(
        setpoints=unsafe_points,
        selected_edges=(('robot_10', 'robot_2'),),
        solver_status='optimal',
        solve_duration_sec=0.125,
        created_at=now,
    )
    valid = ControllerResult(
        setpoints={
            robot_id: state.position
            for robot_id, state in snapshot.robots.items()
        },
        solver_status='optimal',
        solve_duration_sec=0.100,
        created_at=now,
    )

    class FakeDispatcher:
        faulted = False
        commanded_robot_ids = ()

        def __init__(self):
            self.pending_updates = []

        def update_pending(self, points):
            self.pending_updates.append(points)

    dispatcher = FakeDispatcher()
    counters = {'dispatch': 0, 'visualization': 0, 'diagnostics': 0}
    coordinator = SimpleNamespace(
        config=example_config,
        controller=SimpleNamespace(compute=lambda _snapshot: rejected),
        dispatcher=dispatcher,
        latest_result=None,
        latest_rejected_result=None,
        latest_snapshot=None,
        latest_stop_reason='',
        _snapshot=lambda: snapshot,
        _publish_visualization=lambda *_args: counters.__setitem__(
            'visualization', counters['visualization'] + 1
        ),
        _publish_diagnostics=lambda: counters.__setitem__(
            'diagnostics', counters['diagnostics'] + 1
        ),
        _publish_controller_telemetry=lambda *_args: None,
        _dispatch=lambda _snapshot: counters.__setitem__(
            'dispatch', counters['dispatch'] + 1
        ),
    )
    coordinator._compute_valid_result = lambda selected_snapshot: (
        SwarmCoordinator._compute_valid_result(coordinator, selected_snapshot)
    )

    SwarmCoordinator._control_cycle(coordinator)

    assert coordinator.latest_result is not None
    assert coordinator.latest_rejected_result is None
    assert len(dispatcher.pending_updates) == 1
    assert counters['dispatch'] == 1
    assert counters['visualization'] == 1
    assert coordinator.latest_stop_reason == ''

    coordinator.controller = SimpleNamespace(compute=lambda _snapshot: valid)
    SwarmCoordinator._control_cycle(coordinator)

    assert coordinator.latest_stop_reason == ''
    assert coordinator.latest_result.setpoints == valid.setpoints
    assert coordinator.latest_rejected_result is None
    assert len(dispatcher.pending_updates) == 2
    assert counters['dispatch'] == 2
    assert counters['visualization'] == 2


def test_rejected_solver_metadata_is_labeled_rejected_in_diagnostics():
    rejected = ControllerResult(
        setpoints={'131': (0.0, 0.0)},
        selected_edges=(('131', 'station_0'),),
        solver_status='optimal_inaccurate',
        solve_duration_sec=0.125,
        created_at=10.0,
    )

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    publisher = Publisher()
    coordinator = SimpleNamespace(
        config=SimpleNamespace(
            controller=SimpleNamespace(algorithm='convex'),
            safety=SimpleNamespace(dry_run=True),
        ),
        latest_result=None,
        latest_rejected_result=rejected,
        latest_snapshot=None,
        latest_stop_reason='safety rejected result',
        latest_handoff='none',
        aggregator=SimpleNamespace(pose_ages=lambda _now: {}),
        dispatcher=SimpleNamespace(
            commanded_robot_ids=(),
            states={},
            faulted=False,
            observability=lambda _now: {},
        ),
        diagnostics_publisher=publisher,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
    )

    SwarmCoordinator._publish_diagnostics(coordinator)

    values = {
        item.key: item.value
        for item in publisher.messages[0].status[0].values
    }
    assert values['controller_result_state'] == 'rejected'
    assert values['solver_status'] == 'optimal_inaccurate'
    assert values['solve_duration_sec'] == '0.125000'
    assert values['selected_edges'] == "(('131', 'station_0'),)"
    assert values['dispatch_state'] == 'dry_run'


def test_overdue_active_waypoint_is_diagnostic_warning_not_fault(
    example_config, snapshot, monkeypatch
):
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    config = replace(
        example_config,
        safety=replace(example_config.safety, dry_run=False),
        waypoint_dispatch=replace(
            example_config.waypoint_dispatch,
            active_waypoint_warning_sec=1.0,
        ),
    )
    dispatcher = WaypointDispatcher(snapshot.robots, config.waypoint_dispatch)
    dispatcher.update_pending({
        robot_id: (3.0, 3.0) for robot_id in snapshot.robots
    })
    dispatcher.tick(snapshot, 10.0, commands_enabled=True)
    publisher = Publisher()
    coordinator = SimpleNamespace(
        config=config,
        latest_result=None,
        latest_rejected_result=None,
        latest_snapshot=snapshot,
        latest_stop_reason='',
        latest_handoff='waypoint',
        aggregator=SimpleNamespace(
            pose_ages=lambda _now: {
                robot_id: 0.1 for robot_id in snapshot.robots
            }
        ),
        dispatcher=dispatcher,
        diagnostics_publisher=publisher,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
    )
    monkeypatch.setattr(coordinator_module.time, 'monotonic', lambda: 11.1)

    SwarmCoordinator._publish_diagnostics(coordinator)

    status = publisher.messages[0].status[0]
    values = {item.key: item.value for item in status.values}
    assert status.level == DiagnosticStatus.WARN
    assert status.message.startswith('active waypoint overdue:')
    assert values['active_waypoint_overdue_robot_2'] == 'True'
    assert float(values['active_waypoint_age_sec_robot_2']) > 1.0
    assert not dispatcher.faulted
