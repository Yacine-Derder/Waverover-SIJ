from dataclasses import replace

from waverover_swarm_controller.controllers.heuristic import HeuristicController
from waverover_swarm_controller.controllers.mpc_distributed import (
    select_fiedler_edges,
)
from waverover_swarm_controller.models import SwarmSnapshot


def test_zero_robots_and_zero_targets_are_safe(example_config, snapshot):
    empty_robots = replace(snapshot, robots={})
    assert HeuristicController(example_config).compute(empty_robots).setpoints == {}

    empty_targets = replace(snapshot, targets={})
    result = HeuristicController(example_config).compute(empty_targets)
    assert set(result.setpoints) == set(snapshot.robots)
    assert all(point == snapshot.station.position for point in result.setpoints.values())

    no_station = replace(snapshot, station=None)
    result = HeuristicController(example_config).compute(no_station)
    assert result.solver_status == 'missing_station'
    assert result.setpoints == {}


def test_disconnected_fiedler_graph_has_no_nan_or_edges(snapshot):
    far = SwarmSnapshot(
        frame_id=snapshot.frame_id,
        robots={
            key: replace(state, x=100.0 * (index + 1), y=0.0)
            for index, (key, state) in enumerate(snapshot.robots.items())
        },
        targets=snapshot.targets,
        station=snapshot.station,
        created_at=snapshot.created_at,
    )

    assert select_fiedler_edges(far, maximum_range=1.0) == ()


def test_connected_fiedler_edges_use_stable_string_ids(snapshot):
    edges = select_fiedler_edges(snapshot, maximum_range=2.0)
    valid = set(snapshot.robots) | {snapshot.station.station_id}

    assert edges
    assert all(first in valid and second in valid for first, second in edges)
