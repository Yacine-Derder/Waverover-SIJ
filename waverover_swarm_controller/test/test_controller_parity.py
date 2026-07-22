from dataclasses import replace
import math

import pytest

from waverover_swarm_controller.controllers import controller_from_config
from waverover_swarm_controller.controllers.base import repair_controller_result
from waverover_swarm_controller.controllers.heuristic import HeuristicController
from waverover_swarm_controller.controllers.heuristic_decentralized import (
    DecentralizedHeuristicController,
)
from waverover_swarm_controller.models import (
    ControllerResult, RobotState, SwarmSnapshot, TargetState,
)


def with_algorithm(config, algorithm):
    return replace(
        config,
        controller=replace(config.controller, algorithm=algorithm),
    )


@pytest.mark.parametrize(
    'algorithm',
    [
        'heuristic',
        'heuristic_decentralized',
        'convex',
        'mpc_centralized',
        'mpc_distributed',
    ],
)
def test_available_controllers_are_deterministic_and_id_safe(
    algorithm, example_config, snapshot
):
    config = with_algorithm(example_config, algorithm)
    first_controller = controller_from_config(config)
    available, reason = first_controller.availability()
    if not available:
        pytest.skip(reason)
    first = first_controller.compute(snapshot)

    reversed_snapshot = SwarmSnapshot(
        frame_id=snapshot.frame_id,
        robots=dict(reversed(tuple(snapshot.robots.items()))),
        targets=dict(reversed(tuple(snapshot.targets.items()))),
        station=snapshot.station,
        created_at=snapshot.created_at,
    )
    second = controller_from_config(config).compute(reversed_snapshot)

    assert tuple(first.setpoints) == tuple(sorted(snapshot.robots))
    assert first.setpoints == second.setpoints
    assert all(
        len(point) == 2 and all(math.isfinite(value) for value in point)
        for point in first.setpoints.values()
    )
    valid_nodes = set(snapshot.robots) | {snapshot.station.station_id}
    assert all(
        first_id in valid_nodes and second_id in valid_nodes
        for first_id, second_id in first.selected_edges
    )
    if algorithm.startswith('mpc_'):
        assert all(len(path) >= 2 for path in first.predicted_paths.values())


def test_central_heuristic_matches_small_reference_chain(
    example_config, snapshot
):
    result = HeuristicController(example_config).compute(snapshot)

    rounded = {
        tuple(round(value, 6) for value in point)
        for point in result.setpoints.values()
    }
    # Two main-target relay slots remain, while the surplus rover holds its
    # measured position instead of duplicating the fixed station endpoint.
    assert {(1.25, 0.0), (2.5, 0.0)} <= rounded
    assert (0.0, 0.0) not in rounded


def test_relay_count_handles_coincident_and_short_targets(example_config):
    controller = HeuristicController(example_config)

    assert controller.optimal_relay_count(0.0) == 0
    assert controller.optimal_relay_count(0.01) >= 1


def test_distributed_mpc_allocates_weighted_targets_across_collinear_team(
    example_config, snapshot
):
    config = with_algorithm(example_config, 'mpc_distributed')
    robots = {
        key: RobotState(key, x, 0.0, 0.0, snapshot.created_at)
        for key, x in (('r1', 0.5), ('r2', 1.0), ('r3', 1.5))
    }
    targets = {
        'high': TargetState('high', 3.0, 0.0, 10.0),
        'background': TargetState('background', 0.0, 3.0, 1.0),
    }
    selected = replace(snapshot, robots=robots, targets=targets)

    result = controller_from_config(config).compute(selected)
    agents = result.controller_diagnostics['agents']

    assert agents['r3']['dominant_target_id'] == 'high'
    assert agents['r3']['effective_target_coefficients']['high'] > 0.0
    assert any(
        values['effective_target_coefficients']['high'] == 0.0
        for robot_id, values in agents.items() if robot_id != 'r3'
    )
    assert set(result.setpoints) == set(robots)
    assert all(result.predicted_paths[key][1] == result.setpoints[key] for key in robots)

    swapped = replace(selected, targets={
        'high': TargetState('high', 3.0, 0.0, 1.0),
        'background': TargetState('background', 0.0, 3.0, 12.0),
    })
    changed = controller_from_config(config).compute(swapped)
    assert changed.controller_diagnostics['agents']['r1'][
        'effective_target_coefficients'
    ] != agents['r1']['effective_target_coefficients']
    assert changed.setpoints != result.setpoints

    permuted = replace(
        selected,
        robots=dict(reversed(tuple(selected.robots.items()))),
        targets=dict(reversed(tuple(selected.targets.items()))),
    )
    reordered = controller_from_config(config).compute(permuted)
    assert reordered.setpoints == result.setpoints
    assert reordered.controller_diagnostics == result.controller_diagnostics


@pytest.mark.parametrize('algorithm', ['heuristic', 'heuristic_decentralized'])
def test_heuristic_outputs_are_snapped_to_selected_connectivity_links(
    algorithm, example_config, snapshot
):
    config = with_algorithm(example_config, algorithm)
    one_robot = {'r1': RobotState('r1', 0.5, 0.0, 0.0, snapshot.created_at)}
    selected = replace(
        snapshot,
        robots=one_robot,
        targets={'far': TargetState('far', 4.0, 0.0, 10.0, is_main=True)},
    )

    controller = controller_from_config(config)
    controller.compute = lambda chosen: ControllerResult(
        setpoints={'r1': (4.0, 0.0)},
        selected_edges=((chosen.station.station_id, 'r1'),),
        created_at=chosen.created_at,
        target_epoch=chosen.target_epoch,
    )
    result = repair_controller_result(
        config, selected, controller.compute(selected)
    )
    report = result.collision_repair
    allowed = (
        config.communication.maximum_range_m - config.vehicle.turn_radius_m
    )

    assert report['connectivity_edges']
    assert math.dist(result.setpoints['r1'], snapshot.station.position) <= allowed + 1e-6
    assert report['entries']['r1']['original_waypoint'] != result.setpoints['r1']
    assert report['entries']['r1']['snapped_waypoint'] == result.setpoints['r1']
    assert report['maximum_link_violation_m'] <= 1e-6


@pytest.mark.parametrize(
    'controller_type',
    [HeuristicController, DecentralizedHeuristicController],
)
@pytest.mark.parametrize('station_offset', [0.0, 5e-13])
def test_heuristics_keep_station_in_graph_but_never_assign_its_endpoint(
    controller_type, station_offset, example_config, snapshot
):
    station = snapshot.station.position
    targets = {
        'main_target': TargetState(
            'main_target',
            station[0] + station_offset,
            station[1],
            10.0,
            is_main=True,
        )
    }
    selected = replace(snapshot, targets=targets)

    result = controller_type(example_config).compute(selected)

    assert set(result.setpoints) == set(snapshot.robots)
    assert all(
        math.dist(point, station) > 1e-9
        for point in result.setpoints.values()
    )
    assert any(snapshot.station.station_id in edge for edge in result.selected_edges)
