from dataclasses import replace
import math
from pathlib import Path

import pytest
import yaml
from builtin_interfaces.msg import Time

from waverover_swarm_controller.config import ConfigError, load_experiment
from waverover_swarm_controller.synthetic_mcs import (
    SyntheticMCS,
    generate_formation,
    validate_formation,
)
from waverover_swarm_controller.synthetic_motion import (
    SyntheticTrajectory,
    action_primitive,
    integrate_motion,
    yaw_quaternion,
    derive_rover_seed,
)


def config_path():
    return Path(__file__).parents[1] / 'config' / 'experiment.example.yaml'


def test_exact_straight_and_left_right_turn_integration():
    straight = integrate_motion(1.0, 2.0, 0.0, 0.5, 0.0, 2.0)
    assert straight == pytest.approx((2.0, 2.0, 0.0))

    left = integrate_motion(0.0, 0.0, 0.0, 0.3, 2.0, math.pi / 4.0)
    right = integrate_motion(0.0, 0.0, 0.0, 0.3, -2.0, math.pi / 4.0)
    assert left == pytest.approx((0.15, 0.15, math.pi / 2.0))
    assert right == pytest.approx((0.15, -0.15, -math.pi / 2.0))


def test_calibrated_primitives_are_forward_and_match_turn_radius():
    config = load_experiment(config_path())
    for action in ('straight', 'bank_left', 'bank_right'):
        speed, yaw_rate = action_primitive(config.vehicle, action)
        assert speed > 0.0
        if yaw_rate:
            assert abs(speed / yaw_rate) == pytest.approx(
                config.vehicle.turn_radius_m, rel=0.01
            )


def test_heading_quaternion_is_normalized_and_correct():
    quaternion = yaw_quaternion(math.pi / 2.0)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)
    assert quaternion == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))


def test_same_seed_reproduces_noisy_true_and_observed_trajectory():
    config = load_experiment(config_path())
    synthetic = replace(
        config.synthetic_mcs,
        mode='noisy_path',
        seed=1234,
        process_speed_std_mps=0.01,
        process_yaw_rate_std_rad_s=0.02,
        measurement_position_std_m=0.005,
        measurement_heading_std_rad=0.01,
    )
    config = replace(config, synthetic_mcs=synthetic)
    positions = generate_formation(config.robot_ids, config.station.position, 0.5)
    first = SyntheticTrajectory(config, positions, 20.0)
    second = SyntheticTrajectory(config, positions, 20.0)

    first_values = []
    second_values = []
    for _index in range(20):
        first_values.append((first.step(), first.observed_formation(lambda _p: True)))
        second_values.append((second.step(), second.observed_formation(lambda _p: True)))

    assert first_values == second_values
    assert first.metadata() == second.metadata()


def test_different_or_entropy_seeds_change_noisy_trajectory_metadata():
    config = load_experiment(config_path())
    config = replace(
        config,
        synthetic_mcs=replace(
            config.synthetic_mcs,
            mode='random_walk',
            seed=None,
            process_speed_std_mps=0.01,
        ),
    )
    positions = generate_formation(config.robot_ids, config.station.position, 0.5)
    first = SyntheticTrajectory(config, positions, 20.0)
    second = SyntheticTrajectory(config, positions, 20.0)
    assert first.actual_seed != second.actual_seed
    assert first.step() != second.step()


def test_static_mode_preserves_formation_exactly():
    config = load_experiment(config_path())
    positions = generate_formation(config.robot_ids, config.station.position, 0.5)
    trajectory = SyntheticTrajectory(config, positions, 20.0, initial_yaw=0.7)
    result = trajectory.step()
    assert result.positions == positions
    assert all(value == pytest.approx(0.7) for value in result.headings.values())
    assert set(result.actions.values()) == {'static'}
    assert set(result.speeds.values()) == {0.0}
    assert set(result.yaw_rates.values()) == {0.0}


def test_dynamic_rigid_motion_preserves_separation_and_connectivity():
    path = Path(__file__).parents[1] / 'config' / 'dynamic_smoke_test_6.yaml'
    config = load_experiment(path)
    positions = generate_formation(config.robot_ids, config.station.position, 0.5)
    trajectory = SyntheticTrajectory(config, positions, 20.0)
    for _index in range(100):
        result = trajectory.step(lambda values: validate_formation(config, values))
        assert all(speed > 0.0 for speed in result.speeds.values())
        validation = validate_formation(config, result.positions)
        assert validation.minimum_separation_m >= 0.35
        assert validation.algebraic_connectivity > 0.0


def test_invalid_script_and_turn_radius_are_rejected(tmp_path):
    source = yaml.safe_load(config_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(Path(__file__).parents[1] / 'config' / 'targets.yaml')
    source['synthetic_mcs'] = {
        'mode': 'scripted',
        'script': [{'action': 'reverse', 'duration_sec': 1.0}],
    }
    experiment = tmp_path / 'experiment.yaml'
    experiment.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='action'):
        load_experiment(experiment)

    source['synthetic_mcs']['script'][0]['action'] = 'straight'
    source['vehicle']['turn_radius_m'] = 1.0
    experiment.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='inconsistent'):
        load_experiment(experiment)


def test_metadata_contains_seed_noise_calibration_and_segments():
    config = load_experiment(
        Path(__file__).parents[1] / 'config' / 'dynamic_smoke_test_6.yaml'
    )
    positions = generate_formation(config.robot_ids, config.station.position, 0.5)
    trajectory = SyntheticTrajectory(config, positions, 20.0)
    metadata = trajectory.metadata()
    assert metadata['schema_version'] == 2
    assert metadata['actual_seed'] == 2026
    assert metadata['mode'] == 'noisy_path'
    assert metadata['vehicle']['straight_speed_mps'] > 0.0
    assert metadata['generated_segments']
    assert metadata['derived_rover_seeds']
    assert metadata['initial_radius_m'] == pytest.approx(0.5)


def test_all_pose_messages_in_a_tick_can_share_one_timestamp():
    stamp = Time(sec=12, nanosec=345)
    messages = [
        SyntheticMCS._pose_message(stamp, 'robotics_lab', (index, 0.0), 0.2)
        for index in range(6)
    ]
    assert all(message.header.stamp == stamp for message in messages)


def independent_config(seed=2026):
    config = load_experiment(
        Path(__file__).parents[1] / 'config' / 'dynamic_smoke_test_6.yaml'
    )
    return replace(
        config,
        synthetic_mcs=replace(config.synthetic_mcs, seed=seed),
    )


def test_independent_streams_are_per_id_reproducible_and_order_independent():
    config = independent_config()
    positions = generate_formation(config.robot_ids, (0.0, 0.0), 1.0)
    reordered = dict(reversed(tuple(positions.items())))
    first = SyntheticTrajectory(config, positions, 20.0)
    second = SyntheticTrajectory(config, reordered, 20.0)
    first_values = []
    second_values = []
    for _index in range(40):
        first_values.append(first.step())
        second_values.append(second.step())
    assert first_values == second_values
    assert len(set(first.derived_seeds.values())) == len(config.robot_ids)
    assert derive_rover_seed(2026, '131') != derive_rover_seed(2026, '132')
    assert len({
        tuple(result.positions[robot_id] for result in first_values)
        for robot_id in config.robot_ids
    }) == len(config.robot_ids)


def test_different_master_seeds_change_independent_trajectories():
    first_config = independent_config(1)
    second_config = independent_config(2)
    positions = generate_formation(first_config.robot_ids, (0.0, 0.0), 1.0)
    first = SyntheticTrajectory(first_config, positions, 20.0)
    second = SyntheticTrajectory(second_config, positions, 20.0)
    assert [first.step() for _ in range(20)] != [second.step() for _ in range(20)]


def test_independent_motion_is_finite_forward_atomic_and_safe():
    config = independent_config()
    positions = generate_formation(config.robot_ids, (0.0, 0.0), 1.0)
    trajectory = SyntheticTrajectory(config, positions, 20.0)
    pair_distances = []
    lambda_values = []
    previous_positions = dict(positions)
    for _index in range(250):
        result = trajectory.step(
            lambda values: validate_formation(config, values)
        )
        validation = trajectory.last_true_validation
        assert validation.minimum_separation_m >= 0.35
        assert all(config.safety.geofence.contains(point)
                   for point in result.positions.values())
        assert all(math.isfinite(value) for state in trajectory.states.values()
                   for value in (state.x, state.y, state.yaw,
                                 state.last_speed_mps, state.last_yaw_rate_rad_s))
        assert all(speed > 0.0 for speed in result.speeds.values())
        assert set(result.actions.values()) <= {
            'straight', 'bank_left', 'bank_right'
        }
        maximum_step = (
            max(config.vehicle.straight_speed_mps,
                config.vehicle.turning_path_speed_mps)
            + 3.0 * config.synthetic_mcs.process_speed_std_mps
        ) * trajectory.timestep
        assert all(
            math.dist(previous_positions[robot_id], point)
            <= maximum_step + 1e-9
            for robot_id, point in result.positions.items()
        )
        for robot_id, action in result.actions.items():
            if action == 'straight':
                assert abs(result.yaw_rates[robot_id]) <= (
                    3.0 * config.synthetic_mcs.process_yaw_rate_std_rad_s
                    + 1e-12
                )
            else:
                radius = abs(
                    result.speeds[robot_id] / result.yaw_rates[robot_id]
                )
                assert radius == pytest.approx(
                    config.vehicle.turn_radius_m, abs=0.01
                )
        previous_positions = dict(result.positions)
        pair_distances.append(validation.minimum_separation_m)
        lambda_values.append(validation.binary_lambda_2)
    assert max(pair_distances) - min(pair_distances) > 0.01
    assert max(lambda_values) - min(lambda_values) > 0.01
    assert len(set(result.headings.values())) > 1
    assert len(set(result.speeds.values())) > 1
    assert trajectory.corrective_interventions > 0


def test_unsafe_transition_does_not_partially_commit_and_retries_fail_closed():
    config = independent_config()
    config = replace(
        config,
        synthetic_mcs=replace(
            config.synthetic_mcs, maximum_transition_attempts=3
        ),
    )
    positions = generate_formation(config.robot_ids, (0.0, 0.0), 1.0)
    trajectory = SyntheticTrajectory(config, positions, 20.0)
    before = trajectory.states
    with pytest.raises(ValueError, match='after 3 attempts'):
        trajectory.step(lambda _values: (_ for _ in ()).throw(
            ValueError('forced unsafe candidate')
        ))
    assert trajectory.states == before
    assert trajectory.elapsed == 0.0


def test_measurement_rejection_never_changes_true_state():
    config = independent_config()
    positions = generate_formation(config.robot_ids, (0.0, 0.0), 1.0)
    trajectory = SyntheticTrajectory(config, positions, 20.0)
    trajectory.step(lambda values: validate_formation(config, values))
    before = trajectory.states
    with pytest.raises(ValueError, match='observation'):
        trajectory.observed_formation(
            lambda _values: (_ for _ in ()).throw(ValueError('reject noise')),
            maximum_attempts=2,
        )
    assert trajectory.states == before
