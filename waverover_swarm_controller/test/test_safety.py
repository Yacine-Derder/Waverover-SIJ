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


def test_current_separation_error_names_deterministically_ordered_ids(
    example_config, snapshot
):
    close_snapshot = replace(
        snapshot,
        robots={
            **snapshot.robots,
            'robot_10': replace(snapshot.robots['robot_10'], x=0.26, y=0.1),
        },
    )
    with pytest.raises(SafetyViolation) as caught:
        validate_controller_result(
            example_config, close_snapshot, safe_result(close_snapshot), 10.1
        )
    assert str(caught.value) == (
        'Emergency current separation between robot_10 and robot_2 is 0.010 m, '
        'below 0.350 m.'
    )


def test_immediate_separation_error_names_deterministically_ordered_ids(
    example_config, snapshot
):
    points = {
        'robot_30': snapshot.robots['robot_30'].position,
        'robot_2': (1.01, 1.0),
        'robot_10': (1.0, 1.0),
    }
    with pytest.raises(SafetyViolation) as caught:
        validate_controller_result(
            example_config,
            snapshot,
            ControllerResult(setpoints=points, created_at=10.0),
            10.1,
        )
    assert str(caught.value) == (
        'Immediate predicted separation between robot_10 and robot_2 is '
        '0.010 m, below 0.350 m.'
    )


def test_predicted_separation_error_names_ids_and_path_step(
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
    with pytest.raises(SafetyViolation) as caught:
        validate_controller_result(
            example_config,
            snapshot,
            ControllerResult(
                setpoints={key: state.position for key, state in snapshot.robots.items()},
                predicted_paths=paths,
                created_at=10.0,
            ),
            10.1,
        )
    assert str(caught.value) == (
        'Predicted separation between robot_10 and robot_2 at path step 2 is '
        '0.000 m, below 0.350 m.'
    )


def test_predicted_first_future_point_must_match_setpoint(
    example_config, snapshot
):
    paths = {
        key: (state.position, (state.x + 0.01, state.y))
        for key, state in snapshot.robots.items()
    }
    with pytest.raises(SafetyViolation, match='step 1.*does not match'):
        validate_controller_result(
            example_config,
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
