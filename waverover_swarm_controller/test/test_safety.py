from dataclasses import replace

import pytest

from waverover_swarm_controller.models import ControllerResult
from waverover_swarm_controller.safety import (
    SafetyViolation,
    validate_controller_result,
)


def safe_result(snapshot, created_at=10.0):
    return ControllerResult(
        setpoints={key: state.position for key, state in snapshot.robots.items()},
        created_at=created_at,
        solver_status='test',
    )


def test_valid_complete_result_passes(example_config, snapshot):
    assert validate_controller_result(
        example_config, snapshot, safe_result(snapshot), 10.1
    )


def test_missing_output_stale_result_and_geofence_stop(example_config, snapshot):
    result = ControllerResult(
        setpoints={'robot_2': (1.0, 0.0)}, created_at=10.0
    )
    with pytest.raises(SafetyViolation, match='mismatch'):
        validate_controller_result(example_config, snapshot, result, 10.1)

    with pytest.raises(SafetyViolation, match='stale'):
        validate_controller_result(
            example_config, snapshot, safe_result(snapshot, 1.0), 10.0
        )

    points = {key: state.position for key, state in snapshot.robots.items()}
    points['robot_2'] = (100.0, 0.0)
    with pytest.raises(SafetyViolation, match='geofence'):
        validate_controller_result(
            example_config,
            snapshot,
            ControllerResult(setpoints=points, created_at=10.0),
            10.1,
        )


def test_current_pose_separation_is_not_shared_command_validation(
    example_config, snapshot
):
    close_snapshot = replace(
        snapshot,
        robots={
            **snapshot.robots,
            'robot_10': replace(snapshot.robots['robot_10'], x=0.26, y=0.1),
        },
    )
    assert validate_controller_result(
        example_config, close_snapshot, safe_result(close_snapshot), 10.1
    ) is True


def test_candidate_endpoint_separation_waits_for_activation_boundary(
    example_config, snapshot
):
    points = {
        'robot_30': snapshot.robots['robot_30'].position,
        'robot_2': (1.01, 1.0),
        'robot_10': (1.0, 1.0),
    }
    assert validate_controller_result(
        example_config, snapshot,
        ControllerResult(setpoints=points, created_at=10.0), 10.1,
    ) is True


def test_unsent_future_path_separation_is_not_shared_validation(
    example_config, snapshot
):
    collision_points = {
        'robot_10': (1.0, 1.0),
        'robot_2': (1.0, 1.0),
        'robot_30': snapshot.robots['robot_30'].position,
    }

    paths = {
        key: (state.position, state.position, collision_points[key])
        for key, state in snapshot.robots.items()
    }
    assert validate_controller_result(
        example_config, snapshot,
        ControllerResult(
            setpoints={key: state.position for key, state in snapshot.robots.items()},
            predicted_paths=paths, created_at=10.0,
        ), 10.1,
    ) is True


def test_predicted_first_future_point_must_match_setpoint(
    example_config, snapshot
):
    paths = {
        key: (state.position, (state.x + 0.01, state.y))
        for key, state in snapshot.robots.items()
    }
    mpc_config = replace(
        example_config,
        controller=replace(
            example_config.controller, algorithm='mpc_centralized'
        ),
    )
    with pytest.raises(SafetyViolation, match='step 1.*does not match'):
        validate_controller_result(
            mpc_config,
            snapshot,
            ControllerResult(
                setpoints={
                    key: state.position for key, state in snapshot.robots.items()
                },
                predicted_paths=paths,
                created_at=10.0,
            ),
            10.1,
        )
