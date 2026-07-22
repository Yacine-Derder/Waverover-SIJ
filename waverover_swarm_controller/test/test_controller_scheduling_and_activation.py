from dataclasses import replace
import math
import time
from types import SimpleNamespace

import pytest

from waverover_swarm_controller.coordinator_node import SwarmCoordinator
from waverover_swarm_controller.models import ControllerResult, TargetState
from waverover_swarm_controller.waypoint_dispatcher import WaypointDispatcher


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class _SchedulingDispatcher:
    faulted = False
    last_activation_failure = ''

    def __init__(self):
        self.updates = []

    def update_pending(
        self, points, target_epoch=0, objective_revision=0, **options
    ):
        self.updates.append(
            (dict(points), target_epoch, objective_revision, options)
        )


def _result(snapshot):
    return ControllerResult(
        setpoints={
            robot_id: state.position
            for robot_id, state in snapshot.robots.items()
        },
        created_at=time.monotonic(),
        target_epoch=snapshot.target_epoch,
    )


def _coordinator(config, snapshots):
    calls = []
    controller = SimpleNamespace(
        compute=lambda snapshot: calls.append(snapshot) or _result(snapshot)
    )
    dispatcher = _SchedulingDispatcher()
    coordinator = SimpleNamespace(
        config=config,
        controller=controller,
        dispatcher=dispatcher,
        latest_collision_events=[],
        latest_rejected_result=None,
        latest_result=None,
        latest_snapshot=None,
        latest_stop_reason='',
        latest_execution_outcome=None,
        fallback_counters={},
        consecutive_recovery_cycles=0,
        _last_controller_mode=None,
        _last_objective_signature=None,
        _objective_revision=0,
        _last_compute_reason='not_computed',
        _controller_compute_count=0,
        _snapshot=lambda: next(snapshots),
        get_logger=lambda: _Logger(),
        _publish_controller_telemetry=lambda *_args: None,
        _publish_visualization=lambda *_args: None,
        _publish_diagnostics=lambda: None,
        _dispatch=lambda _snapshot: None,
    )
    coordinator._compute_valid_result = lambda snapshot: (
        SwarmCoordinator._compute_valid_result(coordinator, snapshot)
    )
    return coordinator, calls, dispatcher


@pytest.mark.parametrize(
    'algorithm', ['heuristic', 'convex']
)
def test_final_destination_controllers_ignore_pose_only_changes(
    algorithm, example_config, snapshot
):
    config = replace(
        example_config,
        controller=replace(example_config.controller, algorithm=algorithm),
    )
    moved = replace(
        snapshot,
        robots={
            robot_id: replace(
                state, x=state.x + 0.2, y=state.y - 0.1,
                receipt_time=state.receipt_time + 1.0,
            )
            for robot_id, state in snapshot.robots.items()
        },
        created_at=snapshot.created_at + 1.0,
    )
    target_id = sorted(snapshot.targets)[0]
    changed_targets = dict(moved.targets)
    changed_targets[target_id] = replace(
        changed_targets[target_id], x=changed_targets[target_id].x + 0.1
    )
    changed = replace(moved, targets=changed_targets)
    coordinator, calls, dispatcher = _coordinator(
        config, iter((snapshot, moved, moved, changed, changed))
    )
    for _index in range(5):
        SwarmCoordinator._control_cycle(coordinator)
    assert len(calls) == 2
    assert len(dispatcher.updates) == 2
    assert coordinator._last_compute_reason == 'mission_objective_changed'
    assert dispatcher.updates[-1][2] == 2


def test_decentralized_heuristic_recomputes_reactively(
    example_config, snapshot
):
    config = replace(
        example_config,
        controller=replace(
            example_config.controller, algorithm='heuristic_decentralized'
        ),
    )
    moved = replace(
        snapshot,
        robots={
            robot_id: replace(state, x=state.x + 0.1)
            for robot_id, state in snapshot.robots.items()
        },
    )
    coordinator, calls, dispatcher = _coordinator(
        config, iter((snapshot, moved, moved))
    )
    for _index in range(3):
        SwarmCoordinator._control_cycle(coordinator)
    assert len(calls) == 3
    assert len(dispatcher.updates) == 3
    assert coordinator._last_compute_reason == 'reactive_periodic'
    assert dispatcher.updates[-1][3]['allow_supersession']
    assert dispatcher.updates[-1][3]['command_revision'] == 3


def test_semantic_target_changes_trigger_once_and_order_does_not(
    example_config, snapshot
):
    coordinator, calls, dispatcher = _coordinator(
        example_config, iter(())
    )
    assert SwarmCoordinator._computation_reason(
        coordinator, snapshot
    ) == 'startup'
    reordered = replace(snapshot, targets=dict(reversed(tuple(
        snapshot.targets.items()
    ))))
    assert SwarmCoordinator._computation_reason(coordinator, reordered) is None
    target_id = sorted(snapshot.targets)[0]
    changed_position = dict(snapshot.targets)
    changed_position[target_id] = replace(
        changed_position[target_id], x=changed_position[target_id].x + 0.1
    )
    changed = replace(snapshot, targets=changed_position)
    assert SwarmCoordinator._computation_reason(
        coordinator, changed
    ) == 'mission_objective_changed'
    assert SwarmCoordinator._computation_reason(coordinator, changed) is None
    changed_weight = dict(changed.targets)
    changed_weight[target_id] = TargetState(
        target_id,
        changed_weight[target_id].x,
        changed_weight[target_id].y,
        changed_weight[target_id].weight + 1.0,
        not changed_weight[target_id].is_priority,
    )
    weighted = replace(
        changed, targets=changed_weight, priority_target_id=target_id
    )
    assert SwarmCoordinator._computation_reason(
        coordinator, weighted
    ) == 'mission_objective_changed'
    assert coordinator._objective_revision == 3
    assert calls == [] and dispatcher.updates == []


@pytest.mark.parametrize('algorithm', ['mpc_centralized', 'mpc_distributed'])
def test_mpc_schedule_remains_periodic(algorithm, example_config, snapshot):
    config = replace(
        example_config,
        controller=replace(example_config.controller, algorithm=algorithm),
    )
    coordinator = SimpleNamespace(
        config=config,
        _last_objective_signature=None,
        _objective_revision=0,
    )
    assert SwarmCoordinator._computation_reason(
        coordinator, snapshot
    ) == 'startup'
    assert SwarmCoordinator._computation_reason(
        coordinator, snapshot
    ) == 'periodic_mpc'
    assert SwarmCoordinator._computation_reason(
        coordinator, snapshot
    ) == 'periodic_mpc'


def _dispatcher(example_config, robot_ids=('a', 'b')):
    return WaypointDispatcher(
        robot_ids, example_config.waypoint_dispatch, example_config.safety
    )


def _snapshot_at(**positions):
    return SimpleNamespace(robots={
        robot_id: SimpleNamespace(position=point)
        for robot_id, point in positions.items()
    })


def test_initial_outgoing_batch_is_endpoint_corrected_and_pose_independent(
    example_config
):
    outputs = []
    for snapshot in (
        _snapshot_at(a=(0.0, 0.0), b=(0.1, 0.0)),
        _snapshot_at(a=(3.0, 3.0), b=(-3.0, -3.0)),
    ):
        dispatcher = _dispatcher(example_config)
        dispatcher.update_pending(
            {'a': (1.0, 1.0), 'b': (1.1, 1.0)}, objective_revision=1
        )
        actions = dispatcher.tick(snapshot, 1.0, True)
        points = {action.robot_id: action.point for action in actions}
        assert len(actions) == 2
        assert math.dist(points['a'], points['b']) >= (
            example_config.safety.minimum_separation_m - 1e-9
        )
        outputs.append(points)
    assert outputs[0] == outputs[1]


def test_endpoints_at_exact_minimum_separation_are_unchanged(example_config):
    dispatcher = _dispatcher(example_config)
    separation = example_config.safety.minimum_separation_m
    outgoing = {'a': (1.0, 1.0), 'b': (1.0 + separation, 1.0)}
    dispatcher.update_pending(outgoing, objective_revision=1)
    actions = dispatcher.tick(_snapshot_at(a=(3, 3), b=(-3, -3)), 1.0, True)
    assert {action.robot_id: action.point for action in actions} == outgoing


def test_stale_objective_revision_cannot_replace_newest_pending(example_config):
    dispatcher = _dispatcher(example_config, ('a',))
    dispatcher.update_pending({'a': (1.0, 0.0)}, objective_revision=3)
    dispatcher.update_pending({'a': (2.0, 0.0)}, objective_revision=2)
    action = dispatcher.tick(_snapshot_at(a=(0.0, 0.0)), 1.0, True)[0]
    assert action.point == (1.0, 0.0)
    assert action.objective_revision == 3


def test_crossing_paths_and_nearby_live_poses_do_not_change_endpoints(
    example_config
):
    dispatcher = _dispatcher(example_config)
    outgoing = {'a': (2.0, 0.0), 'b': (0.0, 0.0)}
    dispatcher.update_pending(outgoing, objective_revision=1)
    actions = dispatcher.tick(
        _snapshot_at(a=(0.0, 0.0), b=(2.0, 0.0)), 1.0, True
    )
    assert {action.robot_id: action.point for action in actions} == outgoing


def test_partial_handoff_fixes_active_and_revalidates_ack_promotion(
    example_config
):
    dispatcher = _dispatcher(example_config)
    dispatcher.update_pending(
        {'a': (0.0, 0.0), 'b': (1.0, 0.0)}, objective_revision=1
    )
    first = dispatcher.tick(_snapshot_at(a=(3, 3), b=(-3, -3)), 1.0, True)
    by_id = {action.robot_id: action for action in first}
    dispatcher.update_pending({'a': (0.8, 0.0)}, objective_revision=2)
    actions = dispatcher.acknowledge(
        'a', 'robotics_lab', by_id['a'].token, by_id['a'].point, 2.0,
        'robotics_lab', measured_position=(0.0, 0.0),
    )
    assert dispatcher.states['b'].active_waypoint == (1.0, 0.0)
    assert len(actions) == 1
    assert math.dist(actions[0].point, (1.0, 0.0)) >= (
        example_config.safety.minimum_separation_m - 1e-9
    )


def test_impossible_outgoing_batch_publishes_none(example_config):
    tiny = replace(
        example_config.safety,
        geofence=replace(
            example_config.safety.geofence,
            x_min=0.0, x_max=0.1, y_min=0.0, y_max=0.1,
        ),
    )
    dispatcher = WaypointDispatcher(
        ('a', 'b'), example_config.waypoint_dispatch, tiny
    )
    dispatcher.update_pending(
        {'a': (0.0, 0.0), 'b': (0.1, 0.1)}, objective_revision=1
    )
    assert dispatcher.tick(_snapshot_at(a=(0, 0), b=(0.1, 0.1)), 1, True) == []
    assert dispatcher.last_activation_failure.startswith(
        'outgoing_waypoint_separation_failed'
    )
