from dataclasses import replace
import math
from pathlib import Path
import time

import numpy as np
import pytest

from waverover_swarm_controller.config import load_experiment
from waverover_swarm_controller.controllers import controller_from_config
from waverover_swarm_controller.controllers.mpc_distributed import (
    DistributedMpcController,
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
        config.robot_ids, config.station.position, radius_m=0.5
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


@pytest.mark.parametrize('algorithm', ['convex', 'mpc_centralized'])
def test_centralized_outputs_are_left_unmodified_until_activation(algorithm):
    config = _smoke_config(algorithm)
    snapshot = _converging_snapshot(config)

    result = controller_from_config(config).compute(snapshot)
    assert dict(result.collision_repair) == {}
    assert validate_controller_result(
        config, snapshot, result, time.monotonic()
    )
    if algorithm == 'convex':
        assert result.predicted_paths == {}
        assert any(
            math.dist(result.setpoints[robot_id], snapshot.robots[robot_id].position)
            > config.controller.mpc_max_step_m
            for robot_id in result.setpoints
        )
    else:
        assert {len(path) for path in result.predicted_paths.values()} == {
            1 + config.controller.mpc_horizon
        }
        assert all(
            result.predicted_paths[robot_id][1] == result.setpoints[robot_id]
            for robot_id in result.setpoints
        )


@pytest.mark.parametrize('semantics', ['jacobi', 'gauss_seidel'])
def test_distributed_mpc_semantics_keep_periodic_first_step(semantics):
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
        robot_id, selected_snapshot, edges, neighbor_predictions
    ):
        observations[robot_id] = neighbor_predictions[first_id]
        return original_solve_agent(
            robot_id,
            selected_snapshot,
            edges,
            neighbor_predictions,
        )

    controller._solve_agent = recording_solve_agent
    result = controller.compute(snapshot)

    if semantics == 'jacobi':
        assert observations[second_id] == previous_first_path
    else:
        assert observations[second_id] != previous_first_path
    assert validate_controller_result(
        config, snapshot, result, time.monotonic()
    )
    for robot_id in result.setpoints:
        assert result.predicted_paths[robot_id][1] == result.setpoints[robot_id]


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
