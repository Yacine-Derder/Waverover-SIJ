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
    assert config.waypoint_dispatch.refresh_period_sec == pytest.approx(1.0)
    assert config.waypoint_dispatch.active_waypoint_warning_sec == pytest.approx(
        10.0
    )
    assert config.safety.dry_run
    assert config.synthetic_mcs.mode == 'static'
    assert config.synthetic_mcs.formation_coupling == 'rigid'
    assert config.synthetic_mcs.connectivity_policy == 'enforce'
    assert config.synthetic_mcs.initial_radius_m == pytest.approx(0.5)
    assert config.recording.profile == 'core'
    assert config.analysis.connectivity_alpha == pytest.approx(5.0)


def test_targets_use_neutral_string_ids():
    config = load_experiment(example_path())
    ids = [target.target_id for target in config.targets]

    assert ids == ['target_0', 'target_1', 'target_2', 'target_3']
    assert not any(target.is_priority for target in config.targets)
    assert config.target_dynamics.mode == 'random_priority'


def test_missing_pipeline_sections_keep_backward_compatible_defaults(tmp_path):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(Path(__file__).parents[1] / 'config' / 'targets.yaml')
    source.pop('synthetic_mcs', None)
    source.pop('recording', None)
    source.pop('analysis', None)
    experiment = tmp_path / 'legacy.yaml'
    experiment.write_text(yaml.safe_dump(source), encoding='utf-8')

    config = load_experiment(experiment)

    assert config.synthetic_mcs.mode == 'static'
    assert config.synthetic_mcs.seed is None
    assert config.synthetic_mcs.formation_coupling == 'rigid'
    assert config.synthetic_mcs.connectivity_policy == 'enforce'
    assert config.synthetic_mcs.initial_radius_m == pytest.approx(0.5)
    assert config.recording.profile == 'core'
    assert config.analysis.connectivity_alpha == pytest.approx(5.0)


def test_legacy_active_timeout_is_accepted_as_nonfatal_warning_alias(tmp_path):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(
        Path(__file__).parents[1] / 'config' / 'targets.yaml'
    )
    source['waypoint_dispatch'].pop('active_waypoint_warning_sec')
    source['waypoint_dispatch'].pop('refresh_period_sec')
    source['waypoint_dispatch']['maximum_active_time_sec'] = 12.5
    experiment = tmp_path / 'legacy-dispatch.yaml'
    experiment.write_text(yaml.safe_dump(source), encoding='utf-8')

    config = load_experiment(experiment)

    assert config.waypoint_dispatch.refresh_period_sec == pytest.approx(1.0)
    assert config.waypoint_dispatch.active_waypoint_warning_sec == pytest.approx(
        12.5
    )


def test_duplicate_target_and_outside_geofence_are_rejected(tmp_path):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    targets_path = Path(__file__).parents[1] / 'config' / 'targets.yaml'
    targets = yaml.safe_load(targets_path.read_text(encoding='utf-8'))
    targets['targets'][1]['id'] = 'target_0'
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


@pytest.mark.parametrize('coupling', ['rigid', 'independent'])
@pytest.mark.parametrize('policy', ['enforce', 'observe'])
def test_synthetic_coupling_and_connectivity_policy_are_accepted(
    tmp_path, coupling, policy
):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(
        Path(__file__).parents[1] / 'config' / 'targets.yaml'
    )
    source['synthetic_mcs'].update({
        'formation_coupling': coupling,
        'connectivity_policy': policy,
        'initial_radius_m': 1.25,
        'maximum_transition_attempts': 7,
    })
    path = tmp_path / ('%s-%s.yaml' % (coupling, policy))
    path.write_text(yaml.safe_dump(source), encoding='utf-8')
    config = load_experiment(path)
    assert config.synthetic_mcs.formation_coupling == coupling
    assert config.synthetic_mcs.connectivity_policy == policy
    assert config.synthetic_mcs.initial_radius_m == pytest.approx(1.25)
    assert config.synthetic_mcs.maximum_transition_attempts == 7


@pytest.mark.parametrize(
    'field,value,pattern',
    [
        ('formation_coupling', 'elastic', 'formation_coupling'),
        ('connectivity_policy', 'ignore', 'connectivity_policy'),
        ('initial_radius_m', -1.0, 'initial_radius_m'),
        ('initial_radius_m', float('nan'), 'initial_radius_m'),
        ('maximum_transition_attempts', 0, 'maximum_transition_attempts'),
        ('maximum_transition_attempts', 1.5, 'maximum_transition_attempts'),
    ],
)
def test_invalid_synthetic_transition_configuration_is_rejected(
    tmp_path, field, value, pattern
):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(
        Path(__file__).parents[1] / 'config' / 'targets.yaml'
    )
    source['synthetic_mcs'][field] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match=pattern):
        load_experiment(path)
