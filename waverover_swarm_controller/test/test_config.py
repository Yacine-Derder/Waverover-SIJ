from pathlib import Path

import pytest
import yaml

from waverover_swarm_controller.config import ConfigError, load_experiment


def example_path():
    return Path(__file__).parents[1] / 'config' / 'experiment.example.yaml'


def test_calibrated_defaults_and_pc_robot_ids():
    config = load_experiment(example_path())

    assert config.frame_id == 'robotics_lab'
    assert config.robot_ids == ('131', '132', '133', '134', '135', '136')
    assert config.vehicle.straight_speed_mps == pytest.approx(0.333333)
    assert config.vehicle.turn_radius_m == pytest.approx(0.15)
    assert config.vehicle.bank_yaw_rate_rad_s == pytest.approx(2.513274)
    assert config.vehicle.turning_path_speed_mps == pytest.approx(0.376991)
    assert config.controller.control_period_sec == pytest.approx(1.0)
    assert config.controller.mpc_horizon == 5
    assert config.controller.mpc_max_step_m == pytest.approx(0.333333)
    assert config.controller.minimum_mpc_lookahead_m == pytest.approx(0.30)
    assert config.safety.dry_run


def test_targets_use_string_ids_and_exactly_one_main():
    config = load_experiment(example_path())
    ids = [target.target_id for target in config.targets]

    assert ids == [
        'target_main',
        'target_secondary_1',
        'target_secondary_2',
        'target_secondary_3',
    ]
    assert sum(target.is_main for target in config.targets) == 1


def test_duplicate_target_and_outside_geofence_are_rejected(tmp_path):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    targets_path = Path(__file__).parents[1] / 'config' / 'targets.yaml'
    targets = yaml.safe_load(targets_path.read_text(encoding='utf-8'))
    targets['targets'][1]['id'] = 'target_main'
    local_targets = tmp_path / 'targets.yaml'
    local_targets.write_text(yaml.safe_dump(targets), encoding='utf-8')
    source['targets_file'] = 'targets.yaml'
    experiment = tmp_path / 'experiment.yaml'
    experiment.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='unique'):
        load_experiment(experiment)

    targets['targets'][1]['id'] = 'secondary'
    targets['targets'][1]['position'] = [100.0, 0.0]
    local_targets.write_text(yaml.safe_dump(targets), encoding='utf-8')
    with pytest.raises(ConfigError, match='geofence'):
        load_experiment(experiment)


@pytest.mark.parametrize('algorithm', ['unknown', 'default', 'mpc'])
def test_unknown_algorithm_never_silently_falls_back(algorithm):
    with pytest.raises(ConfigError, match='algorithm'):
        load_experiment(example_path(), algorithm_override=algorithm)
