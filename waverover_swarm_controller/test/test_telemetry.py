from types import SimpleNamespace

from waverover_swarm_controller.models import ControllerResult
from waverover_swarm_controller.telemetry import (
    build_controller_telemetry,
    canonical_json,
)


def test_controller_telemetry_is_versioned_structured_and_canonical(
    example_config, snapshot
):
    setpoints = {
        robot_id: state.position
        for robot_id, state in snapshot.robots.items()
    }
    paths = {
        robot_id: (state.position, state.position)
        for robot_id, state in snapshot.robots.items()
    }
    result = ControllerResult(
        setpoints=setpoints,
        target_assignments={robot_id: 'main_target' for robot_id in setpoints},
        predicted_paths=paths,
        selected_edges=(('station_0', 'robot_2'),),
        solver_status='optimal',
        solve_duration_sec=0.125,
    )
    payload = build_controller_telemetry(
        example_config,
        snapshot,
        result,
        'valid',
        SimpleNamespace(sec=12, nanosec=34),
        False,
        '',
        {robot_id: 0.1 for robot_id in snapshot.robots},
        setpoints,
        {robot_id: None for robot_id in snapshot.robots},
        {
            robot_id: {
                'active_waypoint': setpoints[robot_id],
                'pending_waypoint': None,
                'active_waypoint_age_sec': 12.0,
                'last_publication_monotonic_sec': 50.0,
                'last_publication_age_sec': 0.25,
                'refresh_count': 7,
                'active_waypoint_overdue': True,
            }
            for robot_id in snapshot.robots
        },
        'none',
    )

    assert payload['schema_version'] == 4
    assert payload['result_state'] == 'valid'
    assert payload['commands_enabled'] is False
    assert 'armed' not in payload
    assert payload['target_assignments']['robot_2'] == 'main_target'
    dispatch = payload['waypoint_dispatch']['robot_2']
    assert dispatch['active_waypoint'] == list(setpoints['robot_2'])
    assert dispatch['pending_waypoint'] is None
    assert dispatch['active_waypoint_age_sec'] == 12.0
    assert dispatch['last_publication_monotonic_sec'] == 50.0
    assert dispatch['last_publication_age_sec'] == 0.25
    assert dispatch['refresh_count'] == 7
    assert dispatch['active_waypoint_overdue']
    assert payload['predicted_minimum_separation']['step'] == 0
    assert payload['current_minimum_separation']['pair'] == [
        'robot_10', 'robot_2'
    ]
    assert payload['connectivity']['binary_lambda_2'] >= 0.0
    assert payload['connectivity']['weighted_lambda_2'] >= 0.0
    encoded = canonical_json(payload)
    assert encoded.startswith('{"active_waypoints"')
    assert 'Infinity' not in encoded and 'NaN' not in encoded
