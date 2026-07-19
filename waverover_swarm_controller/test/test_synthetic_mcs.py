from dataclasses import replace
import math
from pathlib import Path

import pytest

from waverover_swarm_controller.config import ConfigError, GeofenceConfig, load_experiment
from waverover_swarm_controller.synthetic_mcs import (
    generate_formation,
    validate_formation,
    validated_parameters,
    resolve_initial_radius,
)


def smoke_config():
    path = Path(__file__).parents[1] / 'config' / 'smoke_test_6.yaml'
    return load_experiment(path)


def test_six_arbitrary_ids_are_sorted_and_circle_spacing_is_deterministic():
    positions = generate_formation(
        ('136', '131', '134', '132', '135', '133'), (0.0, 0.0), 0.5
    )

    assert tuple(positions) == ('131', '132', '133', '134', '135', '136')
    adjacent = [
        math.dist(positions[first], positions[second])
        for first, second in zip(
            tuple(positions), tuple(positions)[1:] + tuple(positions)[:1]
        )
    ]
    assert adjacent == pytest.approx([0.5] * 6)


def test_one_robot_is_exactly_at_station():
    assert generate_formation(('134',), (1.25, -0.75), 100.0) == {
        '134': (1.25, -0.75)
    }


@pytest.mark.parametrize(
    'values',
    [
        (0.0, 0.5, 0.0, 0.0),
        (-1.0, 0.5, 0.0, 0.0),
        (20.0, -0.1, 0.0, 0.0),
        (float('nan'), 0.5, 0.0, 0.0),
        (20.0, float('inf'), 0.0, 0.0),
    ],
)
def test_invalid_numeric_parameters_are_rejected(values):
    with pytest.raises(ConfigError):
        validated_parameters(*values)


def test_valid_six_rover_formation_reports_metrics():
    config = smoke_config()
    positions = generate_formation(
        config.robot_ids, config.station.position, 0.5
    )

    validation = validate_formation(config, positions)

    assert validation.minimum_separation_m == pytest.approx(0.5)
    assert validation.algebraic_connectivity > 0.0


def test_formation_below_minimum_separation_is_rejected():
    config = smoke_config()
    positions = generate_formation(
        config.robot_ids, config.station.position, 0.1
    )

    with pytest.raises(ConfigError, match='minimum separation'):
        validate_formation(config, positions)


def test_formation_outside_geofence_is_rejected():
    config = smoke_config()
    positions = dict(generate_formation(
        config.robot_ids, config.station.position, 0.5
    ))
    positions['131'] = (10.0, 0.0)

    with pytest.raises(ConfigError, match='outside the geofence'):
        validate_formation(config, positions)


def test_disconnected_formation_is_rejected():
    config = smoke_config()
    wide_geofence = GeofenceConfig(-20.0, 20.0, -20.0, 20.0)
    config = replace(
        config,
        communication=replace(config.communication, maximum_range_m=0.4),
        safety=replace(config.safety, geofence=wide_geofence),
    )
    positions = generate_formation(
        config.robot_ids, config.station.position, 1.0
    )

    with pytest.raises(ConfigError, match='disconnected'):
        validate_formation(config, positions)


def test_observe_policy_accepts_and_reports_disconnected_formation():
    config = smoke_config()
    config = replace(
        config,
        communication=replace(config.communication, maximum_range_m=0.4),
        safety=replace(
            config.safety, geofence=GeofenceConfig(-20.0, 20.0, -20.0, 20.0)
        ),
        synthetic_mcs=replace(
            config.synthetic_mcs, connectivity_policy='observe'
        ),
    )
    positions = generate_formation(config.robot_ids, (0.0, 0.0), 1.0)
    validation = validate_formation(config, positions)
    assert validation.disconnected
    assert validation.binary_lambda_2 == pytest.approx(0.0, abs=1e-12)
    assert validation.connected_components > 1


def test_radius_override_precedence():
    config = smoke_config()
    config = replace(
        config,
        synthetic_mcs=replace(config.synthetic_mcs, initial_radius_m=1.25),
    )
    assert resolve_initial_radius(config, '') == pytest.approx(1.25)
    assert resolve_initial_radius(config, None) == pytest.approx(1.25)
    assert resolve_initial_radius(config, '0.75') == pytest.approx(0.75)
