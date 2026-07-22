from dataclasses import replace
import math

import pytest

from waverover_swarm_controller.controllers.heuristic import (
    HeuristicController,
)
from waverover_swarm_controller.controllers.heuristic_decentralized import (
    DecentralizedHeuristicController,
)
from waverover_swarm_controller.models import (
    RobotState, StationState, SwarmSnapshot, TargetState,
)


def _config(example_config, count=3):
    return replace(
        example_config,
        robot_ids=tuple(chr(ord('a') + index) for index in range(count)),
        controller=replace(
            example_config.controller, algorithm='heuristic_decentralized'
        ),
    )


def _snapshot(positions, targets=None, priority='main'):
    targets = targets or {
        'main': TargetState('main', 3.5, 0.0, 10.0, True)
    }
    return SwarmSnapshot(
        frame_id='robotics_lab',
        robots={
            robot_id: RobotState(robot_id, point[0], point[1], 0.0, 1.0)
            for robot_id, point in positions.items()
        },
        targets=targets,
        station=StationState('station', 0.0, 0.0),
        created_at=1.0,
        priority_target_id=priority,
    )


def _advance(snapshot, result):
    return replace(
        snapshot,
        robots={
            robot_id: replace(
                snapshot.robots[robot_id],
                x=result.setpoints[robot_id][0],
                y=result.setpoints[robot_id][1],
                receipt_time=snapshot.created_at + 1.0,
            )
            for robot_id in snapshot.robots
        },
        created_at=snapshot.created_at + 1.0,
    )


def test_single_line_progressively_converges_to_successive_links(
    example_config
):
    config = _config(example_config)
    snapshot = _snapshot({'a': (0.1, 0.0), 'b': (0.2, 0.0), 'c': (0.3, 0.0)})
    controller = DecentralizedHeuristicController(config)
    initial_terminal_distance = math.dist(
        snapshot.robots['c'].position, snapshot.targets['main'].position
    )
    for _cycle in range(30):
        result = controller.compute(snapshot)
        snapshot = _advance(snapshot, result)

    ordered = sorted(
        (point[0], robot_id) for robot_id, point in result.setpoints.items()
    )
    chain = [(0.0, 0.0)] + [result.setpoints[key] for _, key in ordered]
    distances = [
        math.dist(first, second) for first, second in zip(chain, chain[1:])
    ]
    assert all(
        distance <= controller.safe_link_distance_m + 1e-9
        for distance in distances
    )
    assert ordered[-1][0] > controller.safe_link_distance_m
    assert math.dist(
        chain[-1], snapshot.targets['main'].position
    ) < initial_terminal_distance

    centralized_config = replace(
        config,
        controller=replace(config.controller, algorithm='heuristic'),
    )
    centralized = HeuristicController(centralized_config).compute(snapshot)
    decentralized_x = sorted(point[0] for point in result.setpoints.values())
    centralized_x = sorted(point[0] for point in centralized.setpoints.values())
    assert decentralized_x == pytest.approx(centralized_x, abs=5e-4)


def test_one_robot_line_is_capped_from_station(example_config):
    config = _config(example_config, 1)
    snapshot = _snapshot({'a': (0.1, 0.0)})
    controller = DecentralizedHeuristicController(config)
    point = controller.compute(snapshot).setpoints['a']
    assert math.dist(snapshot.station.position, point) == pytest.approx(
        controller.safe_link_distance_m
    )


def test_local_view_excludes_out_of_range_and_other_cluster_agents(
    example_config
):
    controller = DecentralizedHeuristicController(_config(example_config, 4))
    snapshot = _snapshot({
        'a': (0.5, 0.0), 'b': (0.8, 0.0),
        'c': (3.0, 0.0), 'd': (0.6, 0.1),
    })
    assignment = {'a': 'line', 'b': 'line', 'c': 'line', 'd': 'other'}
    assert tuple(controller._local_view('a', snapshot, assignment)) == ('b',)


def test_out_of_range_pose_cannot_change_local_calculation(example_config):
    controller = DecentralizedHeuristicController(_config(example_config))
    snapshot = _snapshot({'a': (0.6, 0.0), 'b': (0.2, 0.0), 'c': (2.5, 0.0)})
    controller._ensure_plan(snapshot)
    assignment = {
        robot_id: state.cluster_id
        for robot_id, state in controller._agents.items()
    }
    cluster = controller._clusters[0]
    previous = controller._agents
    first, first_details = controller._compute_local_agent(
        'a', snapshot, cluster, assignment, previous
    )
    moved = replace(
        snapshot,
        robots={
            **snapshot.robots,
            'c': replace(snapshot.robots['c'], x=3.4, y=0.4),
        },
    )
    second, second_details = controller._compute_local_agent(
        'a', moved, cluster, assignment, previous
    )
    assert first == second
    assert first_details['local_neighbor_ids'] == ('b',)
    assert second_details['local_neighbor_ids'] == ('b',)


def test_in_range_same_cluster_neighbor_affects_local_waypoint(example_config):
    controller = DecentralizedHeuristicController(_config(example_config))
    snapshot = _snapshot({'a': (0.6, 0.0), 'b': (0.2, 0.0), 'c': (1.0, 0.0)})
    controller._ensure_plan(snapshot)
    assignment = {
        robot_id: state.cluster_id
        for robot_id, state in controller._agents.items()
    }
    cluster = controller._clusters[0]
    first, _details = controller._compute_local_agent(
        'a', snapshot, cluster, assignment, controller._agents
    )
    moved = replace(
        snapshot,
        robots={
            **snapshot.robots,
            # Cross this agent in the local station-to-target ordering.  The
            # local pose (rather than any global pose) must select a different
            # predecessor/successor pair.
            'c': replace(snapshot.robots['c'], x=0.4),
        },
    )
    second, _details = controller._compute_local_agent(
        'a', moved, cluster, assignment, controller._agents
    )
    assert first != second


def test_global_robot_mapping_order_does_not_change_result(example_config):
    positions = {'a': (0.1, 0.0), 'b': (0.2, 0.0), 'c': (0.3, 0.0)}
    forward = _snapshot(positions)
    reverse = replace(
        forward, robots=dict(reversed(tuple(forward.robots.items())))
    )

    first = DecentralizedHeuristicController(
        _config(example_config)
    ).compute(forward)
    second = DecentralizedHeuristicController(
        _config(example_config)
    ).compute(reverse)
    assert first.setpoints == second.setpoints
    assert first.target_assignments == second.target_assignments
    assert first.controller_diagnostics['clusters'] == (
        second.controller_diagnostics['clusters']
    )


def test_other_cluster_pose_cannot_change_local_calculation(example_config):
    controller = DecentralizedHeuristicController(_config(example_config))
    snapshot = _snapshot({'a': (0.6, 0.0), 'b': (0.2, 0.0), 'c': (0.8, 0.0)})
    controller._ensure_plan(snapshot)
    cluster = controller._clusters[0]
    assignment = {'a': cluster.cluster_id, 'b': cluster.cluster_id, 'c': 'other'}
    previous = controller._agents
    first, first_details = controller._compute_local_agent(
        'a', snapshot, cluster, assignment, previous
    )
    moved = replace(
        snapshot,
        robots={
            **snapshot.robots,
            'c': replace(snapshot.robots['c'], x=0.4, y=0.3),
        },
    )
    second, second_details = controller._compute_local_agent(
        'a', moved, cluster, assignment, previous
    )
    assert first == second
    assert first_details['local_neighbor_ids'] == ('b',)
    assert second_details['local_neighbor_ids'] == ('b',)


def test_cluster_reduction_replaces_naive_round_robin(example_config):
    robot_ids = tuple('abcdef')
    config = _config(example_config, len(robot_ids))
    targets = {
        'main': TargetState('main', 3.2, 0.0, 10.0, True),
        't1': TargetState('t1', 3.0, 0.3, 1.0),
        't2': TargetState('t2', 2.8, -0.3, 1.0),
        't3': TargetState('t3', 0.0, 3.2, 1.0),
        't4': TargetState('t4', 0.3, 3.0, 1.0),
        't5': TargetState('t5', -0.3, 2.8, 1.0),
    }
    snapshot = _snapshot(
        {key: (0.1 + 0.05 * index, 0.05 * (index % 2))
         for index, key in enumerate(robot_ids)},
        targets,
    )
    controller = DecentralizedHeuristicController(config)
    result = controller.compute(snapshot)
    clusters = result.controller_diagnostics['clusters']
    assigned_clusters = {
        details['cluster_id']
        for details in result.controller_diagnostics['local_agents'].values()
        if details['cluster_id'] is not None
    }
    assert len(clusters) < len(targets)
    assert len(assigned_clusters) < len(targets)
    assert any(len(values['target_ids']) > 1 for values in clusters.values())
    assert any(values['priority'] for values in clusters.values())


def test_order_and_assignments_persist_through_small_jitter(example_config):
    controller = DecentralizedHeuristicController(_config(example_config))
    snapshot = _snapshot({'a': (0.2, 0.0), 'b': (0.7, 0.0), 'c': (1.2, 0.0)})
    first = controller.compute(snapshot)
    first_revision = first.controller_diagnostics['assignment_revision']
    jittered = replace(
        snapshot,
        robots={
            robot_id: replace(state, x=state.x + (1e-4 if robot_id == 'b' else 0))
            for robot_id, state in snapshot.robots.items()
        },
        created_at=2.0,
    )
    second = controller.compute(jittered)
    assert second.controller_diagnostics['assignment_revision'] == first_revision
    assert {
        key: value['predecessor_id']
        for key, value in first.controller_diagnostics['local_agents'].items()
    } == {
        key: value['predecessor_id']
        for key, value in second.controller_diagnostics['local_agents'].items()
    }


def test_state_reset_starts_new_revisions(example_config):
    controller = DecentralizedHeuristicController(_config(example_config))
    snapshot = _snapshot({'a': (0.1, 0.0), 'b': (0.2, 0.0), 'c': (0.3, 0.0)})
    controller.compute(snapshot)
    controller.reset()
    result = controller.compute(snapshot)
    assert result.controller_diagnostics['cluster_revision'] == 1
    assert result.controller_diagnostics['assignment_revision'] == 1
    assert result.controller_diagnostics['reactive_computation_count'] == 1


def test_objective_change_advances_cluster_revision_once(example_config):
    controller = DecentralizedHeuristicController(_config(example_config))
    snapshot = _snapshot({'a': (0.1, 0.0), 'b': (0.2, 0.0), 'c': (0.3, 0.0)})
    initial = controller.compute(snapshot)
    revision = initial.controller_diagnostics['cluster_revision']
    changed = replace(
        snapshot,
        targets={
            'main': replace(snapshot.targets['main'], x=3.4, y=0.1),
        },
    )
    updated = controller.compute(changed)
    repeated = controller.compute(changed)
    assert updated.controller_diagnostics['cluster_revision'] == revision + 1
    assert repeated.controller_diagnostics['cluster_revision'] == revision + 1
