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


def test_current_and_predicted_separation_violation_stop(
    example_config, snapshot
):
    close_snapshot = replace(
        snapshot,
        robots={
            **snapshot.robots,
            'robot_10': replace(snapshot.robots['robot_10'], x=0.26, y=0.1),
        },
    )
    with pytest.raises(SafetyViolation, match='current separation'):
        validate_controller_result(
            example_config, close_snapshot, safe_result(close_snapshot), 10.1
        )

    paths = {
        key: (state.position, (1.0, 1.0))
        for key, state in snapshot.robots.items()
    }
    with pytest.raises(SafetyViolation, match='Predicted separation'):
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
