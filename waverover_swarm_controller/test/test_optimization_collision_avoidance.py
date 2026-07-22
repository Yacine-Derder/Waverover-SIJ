from dataclasses import replace
from itertools import combinations
import math
from pathlib import Path
import time

import numpy as np
import pytest

from waverover_swarm_controller.config import load_experiment
from waverover_swarm_controller.controllers import controller_from_config
from waverover_swarm_controller.controllers.base import repair_controller_result
from waverover_swarm_controller.controllers.collision_avoidance import (
    centralized_separation_constraints,
    centralized_soft_separation,
    CollisionGeometryError,
    distributed_closing_limits,
    pairwise_geometries,
    SEPARATION_NUMERIC_MARGIN_M,
)
from waverover_swarm_controller.controllers.mpc_distributed import (
    DistributedMpcController,
    select_fiedler_edges,
)
from waverover_swarm_controller.models import (
    RobotState,
    SwarmSnapshot,
    TargetState,
)
from waverover_swarm_controller.safety import validate_controller_result
from waverover_swarm_controller.synthetic_mcs import generate_formation
from waverover_swarm_controller.synthetic_motion import SyntheticTrajectory


def _smoke_config(algorithm, semantics='jacobi'):
    path = Path(__file__).parents[1] / 'config' / 'experiment.yaml'
    config = load_experiment(path, algorithm_override=algorithm)
    return replace(
        config,
        controller=replace(
            config.controller,
            distributed_update_semantics=semantics,
        ),
    )


def _converging_snapshot(config):
    now = time.monotonic()
    positions = generate_formation(
        config.robot_ids, config.station.position, radius_m=1.0
    )
    target = TargetState('common_target', 2.5, 0.0, 10.0, is_main=True)
    return SwarmSnapshot(
        frame_id='robotics_lab',
        robots={
            robot_id: RobotState(robot_id, x, y, 0.0, now)
            for robot_id, (x, y) in positions.items()
        },
        targets={target.target_id: target},
        station=config.station,
        created_at=now,
    )


def _minimum_distance(points):
    return min(
        math.dist(points[first], points[second])
        for first, second in combinations(sorted(points), 2)
    )


def _assert_all_path_steps_safe(result, minimum_separation):
    lengths = {len(path) for path in result.predicted_paths.values()}
    assert len(lengths) == 1
    for step in range(next(iter(lengths))):
        points = {
            robot_id: result.predicted_paths[robot_id][step]
            for robot_id in result.predicted_paths
        }
        assert _minimum_distance(points) >= minimum_separation


@pytest.mark.parametrize('algorithm', ['convex', 'mpc_centralized'])
def test_centralized_six_rover_convergence_is_separation_safe(algorithm):
    config = _smoke_config(algorithm)
    snapshot = _converging_snapshot(config)

    result = repair_controller_result(
        config, snapshot, controller_from_config(config).compute(snapshot)
    )

    assert _minimum_distance(result.setpoints) >= (
        config.safety.minimum_separation_m
    )
    assert _minimum_distance({
        robot_id: path[1]
        for robot_id, path in result.predicted_paths.items()
    }) >= config.safety.minimum_separation_m
    assert result.collision_repair[
        'predicted_paths_after_first_step'
    ] == 'pre_repair'
    assert validate_controller_result(
        config, snapshot, result, time.monotonic()
    )
    expected_length = 2 if algorithm == 'convex' else 1 + config.controller.mpc_horizon
    assert {len(path) for path in result.predicted_paths.values()} == {
        expected_length
    }
    for robot_id in result.setpoints:
        assert result.predicted_paths[robot_id][0] == (
            snapshot.robots[robot_id].position
        )
        assert result.predicted_paths[robot_id][1] == result.setpoints[robot_id]


def test_centralized_separation_problem_is_dcp_and_covers_every_pair_step():
    cp = pytest.importorskip('cvxpy')
    robot_ids = ('131', '132', '133')
    current = {'131': (0.0, 0.0), '132': (0.5, 0.0), '133': (0.0, 0.5)}
    positions = cp.Variable((4, 3, 2))
    constraints = list(centralized_separation_constraints(
        positions, robot_ids, current, minimum_separation=0.35
    ))
    problem = cp.Problem(
        cp.Minimize(cp.sum_squares(positions[1:])),
        [
            positions[0] == np.asarray([current[key] for key in robot_ids]),
            *constraints,
        ],
    )

    assert len(constraints) == 3 * 3
    assert all(constraint.is_dcp() for constraint in constraints)
    assert problem.is_dcp()


def test_soft_separation_slack_keeps_coincident_problem_feasible():
    cp = pytest.importorskip('cvxpy')
    robot_ids = ('131', '132')
    current = {'131': (0.0, 0.0), '132': (0.0, 0.0)}
    positions = cp.Variable((2, 2, 2))
    constraints, penalty = centralized_soft_separation(
        positions, robot_ids, current, 0.35, target_epoch=4
    )
    problem = cp.Problem(
        cp.Minimize(cp.sum_squares(positions[1:]) + penalty),
        [positions[0] == np.zeros((2, 2)), *constraints],
    )
    problem.solve(solver=cp.CLARABEL)
    assert problem.status in ('optimal', 'optimal_inaccurate')


def test_pair_geometry_rejects_unsafe_or_coincident_ids_clearly():
    with pytest.raises(
        CollisionGeometryError, match='131 and 132.*coincident'
    ):
        pairwise_geometries({'132': (0.0, 0.0), '131': (0.0, 0.0)}, 0.35)

    with pytest.raises(CollisionGeometryError, match='131 and 132.*below'):
        pairwise_geometries({'132': (0.34, 0.0), '131': (0.0, 0.0)}, 0.35)

    with pytest.raises(
        CollisionGeometryError, match='closing budget between rover IDs 131 and 132'
    ):
        distributed_closing_limits(
            {'132': (0.3505, 0.0), '131': (0.0, 0.0)}, 0.35
        )


@pytest.mark.parametrize('semantics', ['jacobi', 'gauss_seidel'])
def test_distributed_mpc_semantics_keep_all_pairs_and_steps_safe(semantics):
    config = _smoke_config('mpc_distributed', semantics)
    snapshot = _converging_snapshot(config)
    controller = controller_from_config(config)
    first_id, second_id = sorted(snapshot.robots)[:2]
    previous_first_path = tuple(
        snapshot.robots[first_id].position
        for _ in range(config.controller.mpc_horizon + 1)
    )
    observations = {}
    original_solve_agent = controller._solve_agent

    def recording_solve_agent(
        robot_id, selected_snapshot, edges, neighbor_predictions, closing_limits
    ):
        observations[robot_id] = neighbor_predictions[first_id]
        return original_solve_agent(
            robot_id,
            selected_snapshot,
            edges,
            neighbor_predictions,
            closing_limits,
        )

    controller._solve_agent = recording_solve_agent
    result = controller.compute(snapshot)

    if semantics == 'jacobi':
        assert observations[second_id] == previous_first_path
    else:
        assert observations[second_id] != previous_first_path
    assert _minimum_distance(result.setpoints) >= (
        config.safety.minimum_separation_m
    )
    assert _minimum_distance({
        robot_id: path[1]
        for robot_id, path in result.predicted_paths.items()
    }) >= config.safety.minimum_separation_m
    assert validate_controller_result(
        config, snapshot, result, time.monotonic()
    )
    for robot_id in result.setpoints:
        assert result.predicted_paths[robot_id][1] == result.setpoints[robot_id]

    rover_edges = {
        tuple(sorted(edge))
        for edge in select_fiedler_edges(
            snapshot, config.communication.maximum_range_m
        )
        if config.station.station_id not in edge
    }
    non_fiedler_pairs = [
        pair
        for pair in combinations(sorted(snapshot.robots), 2)
        if pair not in rover_edges
    ]
    assert non_fiedler_pairs
    for first, second in non_fiedler_pairs:
        for step in range(2):
            assert math.dist(
                result.predicted_paths[first][step],
                result.predicted_paths[second][step],
            ) >= config.safety.minimum_separation_m


def test_distributed_limits_cover_non_fiedler_pairs_with_shared_budget():
    config = _smoke_config('mpc_distributed')
    snapshot = _converging_snapshot(config)
    positions = {
        robot_id: state.position
        for robot_id, state in snapshot.robots.items()
    }
    limits = distributed_closing_limits(
        positions, config.safety.minimum_separation_m
    )
    edges = set(select_fiedler_edges(
        snapshot, config.communication.maximum_range_m
    ))

    assert all(len(limits[robot_id]) == len(positions) - 1 for robot_id in limits)
    non_edge = next(
        pair for pair in combinations(sorted(positions), 2)
        if pair not in edges
    )
    first_limit = next(value for value in limits[non_edge[0]] if value[0] == non_edge[1])
    second_limit = next(value for value in limits[non_edge[1]] if value[0] == non_edge[0])
    expected = (
        math.dist(positions[non_edge[0]], positions[non_edge[1]])
        - config.safety.minimum_separation_m
        - SEPARATION_NUMERIC_MARGIN_M
    ) / 2.0
    assert first_limit[2] == pytest.approx(expected)
    assert second_limit[2] == pytest.approx(expected)
    assert second_limit[1] == pytest.approx(
        (-first_limit[1][0], -first_limit[1][1])
    )


def test_distributed_previous_prediction_translates_to_new_measured_pose():
    previous = ((0.0, 0.0), (0.1, 0.2), (0.3, 0.4))

    translated = DistributedMpcController._prediction_at_current(
        previous, (1.0, -2.0), horizon=3
    )

    assert np.asarray(translated) == pytest.approx(np.asarray((
        (1.0, -2.0),
        (1.1, -1.8),
        (1.3, -1.6),
        (1.3, -1.6),
    )))


def test_distributed_mpc_remains_feasible_as_rigid_formation_moves():
    motion_config = load_experiment(
        Path(__file__).parents[1] / 'config' / 'experiment.yaml'
    )
    config = replace(
        motion_config,
        controller=replace(
            motion_config.controller, algorithm='mpc_distributed'
        ),
        synthetic_mcs=replace(
            motion_config.synthetic_mcs,
            formation_coupling='rigid',
            connectivity_policy='enforce',
        ),
    )
    positions = generate_formation(
        config.robot_ids, config.station.position, radius_m=0.5
    )
    trajectory = SyntheticTrajectory(config, positions, rate_hz=20.0)
    controller = controller_from_config(config)

    for _cycle in range(6):
        for _tick in range(20):
            positions, yaw, _action, _speed, _yaw_rate = trajectory.step()
        now = time.monotonic()
        snapshot = SwarmSnapshot(
            frame_id='robotics_lab',
            robots={
                robot_id: RobotState(robot_id, x, y, yaw, now)
                for robot_id, (x, y) in positions.items()
            },
            targets={target.target_id: target for target in config.targets},
            station=config.station,
            created_at=now,
        )
        result = controller.compute(snapshot)
        assert result.solver_status in ('optimal', 'optimal_inaccurate')
        assert validate_controller_result(
            config, snapshot, result, time.monotonic()
        )
