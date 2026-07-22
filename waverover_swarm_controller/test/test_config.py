from pathlib import Path

import pytest

from waverover_swarm_controller.config import (
    ConfigError, load_experiment, SUPPORTED_ALGORITHMS,
)
from waverover_swarm_controller.controllers.base import (
    controller_schedule, ControllerSchedule,
)
import yaml


def example_path():
    return Path(__file__).parents[1] / 'config' / 'experiment.yaml'


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
    assert config.controller.distributed_inter_agent_weight == pytest.approx(2.0)
    assert config.controller.connectivity_recovery_slack_penalty == pytest.approx(
        10000.0
    )
    assert config.waypoint_dispatch.refresh_period_sec == pytest.approx(1.0)
    assert config.waypoint_dispatch.active_waypoint_warning_sec == pytest.approx(
        10.0
    )
    assert config.waypoint_dispatch.repeated_destination_epsilon_m == (
        pytest.approx(0.05)
    )
    assert config.waypoint_dispatch.completed_destination_reissue_distance_m == (
        pytest.approx(0.30)
    )
    assert config.target_dynamics.switch_period_sec == pytest.approx(20.0)
    assert not config.safety.dry_run
    assert config.safety.preferred_separation_m == pytest.approx(0.5)
    assert config.synthetic_mcs.mode == 'random_walk'
    assert config.synthetic_mcs.formation_coupling == 'independent'
    assert config.synthetic_mcs.connectivity_policy == 'observe'
    assert config.synthetic_mcs.initial_radius_m == pytest.approx(1.0)
    assert config.recording.profile == 'core'
    assert config.recording.pose_source == 'mcs'
    assert not config.recording.start_synthetic
    assert config.analysis.connectivity_alpha == pytest.approx(5.0)


def test_targets_use_neutral_string_ids():
    config = load_experiment(example_path())
    ids = [target.target_id for target in config.targets]

    assert ids == [
        'target_0', 'target_1', 'target_2', 'target_3', 'target_4', 'target_5'
    ]
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


def test_absent_target_switch_period_defaults_to_twenty_seconds(tmp_path):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(
        Path(__file__).parents[1] / 'config' / 'targets.yaml'
    )
    source['target_dynamics'].pop('switch_period_sec')
    experiment = tmp_path / 'default-switch.yaml'
    experiment.write_text(yaml.safe_dump(source), encoding='utf-8')

    assert load_experiment(
        experiment
    ).target_dynamics.switch_period_sec == pytest.approx(20.0)


def test_canonical_experiment_uses_best_effort_half_meter_separation():
    config = load_experiment(example_path())
    assert config.target_dynamics.switch_period_sec == pytest.approx(20.0)
    assert config.safety.collision_policy == 'best_effort'
    assert config.safety.preferred_separation_m == pytest.approx(0.5)


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


@pytest.mark.parametrize('algorithm', SUPPORTED_ALGORITHMS)
def test_every_algorithm_uses_the_same_canonical_file(algorithm):
    config = load_experiment(example_path(), algorithm_override=algorithm)
    assert config.configured_algorithm == 'heuristic'
    assert config.controller.algorithm == algorithm
    assert config.algorithm_source == 'cli'
    assert config.safety.minimum_separation_m == pytest.approx(0.5)
    expected = (
        ControllerSchedule.RECEDING_HORIZON
        if algorithm.startswith('mpc_')
        else ControllerSchedule.FINAL_DESTINATION
    )
    assert controller_schedule(config.controller.algorithm) is expected


def test_selected_algorithm_block_isolated_and_schema_is_strict(tmp_path):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(
        Path(__file__).parents[1] / 'config' / 'targets.yaml'
    )
    source['controller']['algorithms']['mpc_distributed'][
        'distributed_inter_agent_weight'
    ] = 91.0
    path = tmp_path / 'isolated.yaml'
    path.write_text(yaml.safe_dump(source), encoding='utf-8')
    assert load_experiment(
        path, algorithm_override='convex'
    ).controller.distributed_inter_agent_weight == pytest.approx(2.0)
    assert load_experiment(
        path, algorithm_override='mpc_distributed'
    ).controller.distributed_inter_agent_weight == pytest.approx(91.0)

    source['controller']['algorithms']['convex']['typo_parameter'] = 1
    path.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='unknown parameter'):
        load_experiment(path)


def test_convex_rejects_stale_mpc_step_parameter(tmp_path):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(
        Path(__file__).parents[1] / 'config' / 'targets.yaml'
    )
    source['controller']['algorithms']['convex']['mpc_max_step_m'] = 0.1
    path = tmp_path / 'stale-convex.yaml'
    path.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='mpc_max_step_m'):
        load_experiment(path, algorithm_override='convex')


def test_missing_selected_parameter_and_unknown_top_level_rejected(tmp_path):
    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['targets_file'] = str(
        Path(__file__).parents[1] / 'config' / 'targets.yaml'
    )
    source['controller']['algorithms']['mpc_centralized'].pop('mpc_horizon')
    path = tmp_path / 'missing.yaml'
    path.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='mpc_horizon'):
        load_experiment(path, algorithm_override='mpc_centralized')

    source = yaml.safe_load(example_path().read_text(encoding='utf-8'))
    source['unexpected'] = True
    path.write_text(yaml.safe_dump(source), encoding='utf-8')
    with pytest.raises(ConfigError, match='unknown parameter'):
        load_experiment(path)


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
