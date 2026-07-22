from dataclasses import replace
import inspect
import math
from pathlib import Path
import time
from types import SimpleNamespace

import pytest
from waverover_swarm_controller.config import ConfigError, load_experiment
from waverover_swarm_controller.controllers.base import (
    deterministic_connectivity_setpoints,
    optimization_hard_link_limit,
)
from waverover_swarm_controller.controllers.convex import ConvexController
from waverover_swarm_controller.controllers.mpc_distributed import (
    DistributedMpcController,
)
from waverover_swarm_controller.coordinator_node import SwarmCoordinator
from waverover_swarm_controller.models import (
    ControllerResult,
    RobotState,
    TargetState,
)
from waverover_swarm_controller.pose_aggregation import SnapshotUnavailableError
import yaml


class Logger:
    def __init__(self):
        self.callsite_severities = {}

    def _log(self, severity):
        callsite = inspect.currentframe().f_back.f_back.f_lineno
        previous = self.callsite_severities.setdefault(callsite, severity)
        if previous != severity:
            raise ValueError('Logger severity cannot be changed between calls.')

    def info(self, _message):
        self._log('info')

    def warning(self, _message):
        self._log('warning')


class Dispatcher:
    faulted = False
    commanded_robot_ids = ()

    def __init__(self):
        self.states = {}
        self.pending_calls = 0

    def update_pending(self, _points, _epoch=0):
        self.pending_calls += 1


def _result(snapshot, mode, status='optimal', slack=0.0, edges=()):
    points = {
        key: state.position for key, state in snapshot.robots.items()
    }
    return ControllerResult(
        setpoints=points,
        predicted_paths={key: (point, point) for key, point in points.items()},
        selected_edges=edges,
        solver_status=status,
        created_at=time.monotonic(),
        target_epoch=snapshot.target_epoch,
        optimization_mode=mode,
        controller_diagnostics={
            'maximum_connectivity_slack_m': slack,
            'total_connectivity_slack_m': slack,
        },
    )


def _coordinator(config, controller):
    logger = Logger()
    return SimpleNamespace(
        config=config,
        controller=controller,
        dispatcher=Dispatcher(),
        latest_collision_events=[],
        latest_rejected_result=None,
        latest_result=None,
        latest_snapshot=None,
        latest_stop_reason='',
        latest_execution_outcome=None,
        fallback_counters={},
        consecutive_recovery_cycles=0,
        _last_controller_mode=None,
        get_logger=lambda: logger,
    )


@pytest.mark.parametrize(
    'algorithm,normal_mode,recovery_mode',
    [
        ('convex', 'normal_convex', 'recovery_convex'),
        ('mpc_centralized', 'normal_mpc', 'recovery_mpc'),
        ('mpc_distributed', 'normal_mpc', 'recovery_mpc'),
    ],
)
def test_normal_and_recovery_hierarchy_modes(
    algorithm, normal_mode, recovery_mode, example_config, snapshot
):
    config = replace(
        example_config,
        controller=replace(example_config.controller, algorithm=algorithm),
    )
    normal = _result(snapshot, normal_mode)
    controller = SimpleNamespace(
        compute=lambda _snapshot: normal,
        compute_recovery=lambda _snapshot: pytest.fail('recovery was called'),
    )
    coordinator = _coordinator(config, controller)

    outcome = SwarmCoordinator._compute_valid_result(coordinator, snapshot)
    assert outcome.controller_mode == normal_mode
    assert outcome.dispatch_allowed
    assert outcome.consecutive_recovery_cycles == 0
    assert 'outcome_reporting_error' not in outcome.failure_metadata

    recovery = _result(snapshot, recovery_mode, slack=0.125)
    controller.compute = lambda _snapshot: (_ for _ in ()).throw(
        RuntimeError('mock infeasible')
    )
    controller.compute_recovery = lambda _snapshot: recovery
    outcome = SwarmCoordinator._compute_valid_result(coordinator, snapshot)
    assert outcome.controller_mode == recovery_mode
    assert outcome.failure_metadata['normal_failure_reason'] == 'mock infeasible'
    assert outcome.failure_metadata['maximum_connectivity_slack_m'] == 0.125
    assert outcome.complete_command_set_generated
    assert outcome.final_command_set_passed_validation
    assert 'outcome_reporting_error' not in outcome.failure_metadata
    controller.compute = lambda _snapshot: normal
    outcome = SwarmCoordinator._compute_valid_result(coordinator, snapshot)
    assert outcome.controller_mode == normal_mode
    assert outcome.consecutive_recovery_cycles == 0
    assert outcome.fallback_counters[recovery_mode] == 1
    assert 'outcome_reporting_error' not in outcome.failure_metadata


def test_outcome_logging_failure_is_non_recursive_and_preserves_cause(
    example_config, snapshot
):
    class BrokenLogger:
        def warning(self, _message):
            raise RuntimeError('reporting failed')

    coordinator = _coordinator(example_config, SimpleNamespace())
    coordinator.get_logger = lambda: BrokenLogger()
    failure = {'controller_exception': {'message': 'solver infeasible'}}
    outcome = SwarmCoordinator._finish_outcome(
        coordinator, None, False, False, False, 'safe_hold', failure
    )
    assert outcome.controller_mode == 'safe_hold'
    assert outcome.failure_metadata['controller_exception']['message'] == (
        'solver infeasible'
    )
    assert outcome.failure_metadata['outcome_reporting_error'] == {
        'type': 'RuntimeError', 'message': 'reporting failed'
    }


def test_real_jazzy_logger_accepts_repeated_mixed_severity_transitions(
    example_config
):
    rclpy_logging = pytest.importorskip('rclpy.logging')
    coordinator = _coordinator(example_config, SimpleNamespace())
    coordinator.get_logger = lambda: rclpy_logging.get_logger(
        'outcome_transition_regression'
    )
    for mode in (
        'normal_convex', 'recovery_convex', 'normal_mpc', 'safe_hold',
        'normal_convex', 'non_dispatching_safe_hold',
    ):
        outcome = SwarmCoordinator._finish_outcome(
            coordinator, None, False, False, False, mode, {}
        )
        assert 'outcome_reporting_error' not in outcome.failure_metadata


def test_actual_convex_recovery_reports_nonnegative_explicit_slack(
    example_config, snapshot
):
    pytest.importorskip('cvxpy')
    config = replace(
        example_config,
        robot_ids=('r1',),
        controller=replace(example_config.controller, algorithm='convex'),
    )
    selected = replace(
        snapshot,
        robots={'r1': RobotState('r1', 3.0, 0.0, 0.0, time.monotonic())},
        targets={'target': TargetState('target', 3.0, 0.0, 10.0)},
    )
    result = ConvexController(config).compute_recovery(selected)
    diagnostics = result.controller_diagnostics
    hard_limit = optimization_hard_link_limit(config)

    assert diagnostics['hard_link_limit_m'] == pytest.approx(hard_limit)
    assert diagnostics['maximum_connectivity_slack_m'] >= 0.0
    assert diagnostics['total_connectivity_slack_m'] >= (
        diagnostics['maximum_connectivity_slack_m']
    )
    assert diagnostics['maximum_connectivity_slack_m'] == pytest.approx(0.0)
    assert math.dist(
        result.setpoints['r1'], selected.station.position
    ) <= hard_limit + diagnostics['maximum_connectivity_slack_m'] + 1e-5


def test_static_connectivity_recovery_is_order_independent_and_not_step_limited(
    example_config, snapshot
):
    robots = {
        'a': RobotState('a', 2.5, 0.0, 0.0, 1.0),
        'b': RobotState('b', 2.5, 2.5, 0.0, 1.0),
    }
    selected = replace(snapshot, robots=robots)
    edges = ((selected.station.station_id, 'a'), ('a', 'b'))

    first, canonical = deterministic_connectivity_setpoints(
        example_config, selected, edges
    )
    second, reversed_edges = deterministic_connectivity_setpoints(
        example_config,
        replace(selected, robots=dict(reversed(tuple(robots.items())))),
        tuple(reversed(edges)),
    )

    assert canonical == reversed_edges
    assert first == second
    assert set(first) == set(robots)
    assert all(all(math.isfinite(value) for value in point) for point in first.values())
    assert any(
        math.dist(first[key], robots[key].position)
        > example_config.controller.mpc_max_step_m + 1e-9
        for key in robots
    )
    assert math.dist(first['a'], selected.station.position) < 2.5
    assert math.dist(first['b'], robots['a'].position) < 2.5


def test_invalid_safe_hold_structurally_cannot_dispatch(example_config, snapshot):
    config = replace(
        example_config,
        controller=replace(example_config.controller, algorithm='convex'),
    )
    outside = replace(
        snapshot,
        robots={
            key: replace(state, x=20.0 + index)
            for index, (key, state) in enumerate(snapshot.robots.items())
        },
    )
    controller = SimpleNamespace(
        compute=lambda _snapshot: (_ for _ in ()).throw(RuntimeError('normal')),
        compute_recovery=lambda _snapshot: (_ for _ in ()).throw(
            RuntimeError('recovery')
        ),
        _last_selected_edges=(),
        _last_solver_status='infeasible',
    )
    coordinator = _coordinator(config, controller)
    dispatch_calls = []
    coordinator._snapshot = lambda: outside
    coordinator._compute_valid_result = lambda selected: (
        SwarmCoordinator._compute_valid_result(coordinator, selected)
    )
    coordinator._publish_controller_telemetry = lambda *_args: None
    coordinator._publish_visualization = lambda *_args: None
    coordinator._publish_diagnostics = lambda: None
    coordinator._dispatch = lambda _snapshot: dispatch_calls.append(True)

    SwarmCoordinator._control_cycle(coordinator)

    outcome = coordinator.latest_execution_outcome
    assert outcome.controller_mode == 'non_dispatching_safe_hold'
    assert not outcome.dispatch_allowed
    assert not outcome.final_command_set_passed_validation
    assert coordinator.dispatcher.pending_calls == 0
    assert dispatch_calls == []
    assert not coordinator.dispatcher.faulted
    assert outcome.fallback_counters['deterministic_recovery'] == 1
    assert outcome.fallback_counters['safe_hold'] == 1
    assert outcome.fallback_counters['non_dispatching_safe_hold'] == 1


def test_pose_unavailable_does_not_stop_or_dispatch_and_can_recover(
    example_config, snapshot
):
    config = replace(
        example_config,
        controller=replace(example_config.controller, algorithm='convex'),
    )
    coordinator = _coordinator(
        config,
        SimpleNamespace(
            compute=lambda selected: _result(selected, 'normal_convex'),
            compute_recovery=lambda _selected: pytest.fail('unexpected'),
        ),
    )
    snapshots = iter((SnapshotUnavailableError('temporarily stale'), snapshot))
    coordinator._snapshot = lambda: (
        (_ for _ in ()).throw(value) if isinstance(value := next(snapshots), Exception)
        else value
    )
    coordinator._compute_valid_result = lambda selected: (
        SwarmCoordinator._compute_valid_result(coordinator, selected)
    )
    coordinator._publish_controller_telemetry = lambda *_args: None
    coordinator._publish_visualization = lambda *_args: None
    coordinator._publish_diagnostics = lambda: None
    dispatch_calls = []
    coordinator._dispatch = lambda _snapshot: dispatch_calls.append(True)

    SwarmCoordinator._control_cycle(coordinator)
    assert coordinator.latest_execution_outcome.controller_mode == 'pose_unavailable'
    assert coordinator.dispatcher.pending_calls == 0
    assert dispatch_calls == []

    SwarmCoordinator._control_cycle(coordinator)
    assert coordinator.latest_execution_outcome.controller_mode == 'normal_convex'
    assert coordinator.dispatcher.pending_calls == 1
    assert dispatch_calls == [True]


def test_distributed_no_selected_rover_edge_uses_closest_rover_objective(
    example_config, snapshot
):
    pytest.importorskip('cvxpy')
    config = replace(
        example_config,
        controller=replace(
            example_config.controller, algorithm='mpc_distributed'
        ),
    )
    controller = DistributedMpcController(config)
    robot_id = sorted(snapshot.robots)[0]
    horizon = config.controller.mpc_horizon
    predictions = {
        key: tuple(state.position for _ in range(horizon + 1))
        for key, state in snapshot.robots.items()
    }
    controller._solve_agent(
        robot_id, snapshot, (), predictions,
    )
    diagnostics = controller._last_agent_diagnostics[robot_id]

    assert diagnostics['selected_neighbors'] == ()
    assert diagnostics['closest_rover_objective_fallback'] is not None
    assert diagnostics['closest_rover_objective_fallback'] in (
        diagnostics['objective_neighbors']
    )


def test_recovery_penalty_defaults_and_rejects_nonpositive(tmp_path):
    source_path = Path(__file__).parents[1] / 'config' / 'experiment.yaml'
    source = yaml.safe_load(source_path.read_text(encoding='utf-8'))
    source['targets_file'] = str(
        Path(__file__).parents[1] / 'config' / 'targets.yaml'
    )
    source['controller']['algorithms']['convex'].pop(
        'connectivity_recovery_slack_penalty'
    )
    legacy = tmp_path / 'legacy.yaml'
    legacy.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='connectivity_recovery_slack_penalty'):
        load_experiment(legacy)

    source['controller']['algorithms']['convex'][
        'connectivity_recovery_slack_penalty'
    ] = 0.0
    invalid = tmp_path / 'invalid.yaml'
    invalid.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='connectivity_recovery_slack_penalty'):
        load_experiment(invalid, algorithm_override='convex')


def test_normal_and_recovery_candidates_are_not_pre_dispatch_repaired(
    example_config, snapshot
):
    config = replace(
        example_config,
        controller=replace(example_config.controller, algorithm='convex'),
    )
    controller = SimpleNamespace(
        compute=lambda selected: _result(selected, 'normal_convex'),
        compute_recovery=lambda selected: _result(selected, 'recovery_convex'),
    )
    coordinator = _coordinator(config, controller)
    outcome = SwarmCoordinator._compute_valid_result(coordinator, snapshot)
    assert dict(outcome.result.collision_repair) == {}

    controller.compute = lambda _selected: (_ for _ in ()).throw(
        RuntimeError('infeasible')
    )
    outcome = SwarmCoordinator._compute_valid_result(coordinator, snapshot)
    assert dict(outcome.result.collision_repair) == {}
