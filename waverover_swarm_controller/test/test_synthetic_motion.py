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
    returned, yaw, action, speed, yaw_rate = trajectory.step()
    assert returned == positions
    assert yaw == pytest.approx(0.7)
    assert (action, speed, yaw_rate) == ('static', 0.0, 0.0)


def test_dynamic_rigid_motion_preserves_separation_and_connectivity():
    path = Path(__file__).parents[1] / 'config' / 'dynamic_smoke_test_6.yaml'
    config = load_experiment(path)
    positions = generate_formation(config.robot_ids, config.station.position, 0.5)
    trajectory = SyntheticTrajectory(config, positions, 20.0)
    for _index in range(100):
        next_positions, _yaw, _action, speed, _yaw_rate = trajectory.step()
        assert speed > 0.0
        validation = validate_formation(config, next_positions)
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
    assert metadata['schema_version'] == 1
    assert metadata['actual_seed'] == 2026
    assert metadata['mode'] == 'noisy_path'
    assert metadata['vehicle']['straight_speed_mps'] > 0.0
    assert metadata['generated_segments']


def test_all_pose_messages_in_a_tick_can_share_one_timestamp():
    stamp = Time(sec=12, nanosec=345)
    messages = [
        SyntheticMCS._pose_message(stamp, 'robotics_lab', (index, 0.0), 0.2)
        for index in range(6)
    ]
    assert all(message.header.stamp == stamp for message in messages)
