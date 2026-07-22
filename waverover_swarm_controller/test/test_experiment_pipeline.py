from datetime import datetime, timezone
import json
from pathlib import Path
import re

import pytest
import yaml

from waverover.stack_config import load_stack_config
from waverover_swarm_controller.analysis_metrics import (
    graph_metrics,
    interpolate_angle,
    main_target_distances,
    mission_cost,
    separation_metrics,
)
from waverover_swarm_controller.config import load_experiment
from waverover_swarm_controller.experiment_recording import (
    atomic_write_yaml,
    create_run_directory,
    qos_overrides,
    recording_topics,
)
from waverover_swarm_controller.replay_run import interpolate_pose
from waverover_swarm_controller.run_experiment import (
    _resolved_run_config,
    parse_arguments,
    resolve_algorithm_selection,
)


def example_config():
    return load_experiment(
        Path(__file__).parents[1] / 'config' / 'experiment.yaml'
    )


def test_recording_profiles_build_safe_topics_for_numeric_ids():
    stack = load_stack_config(require_identity=False)
    core = recording_topics(('134', '7'), stack, 'core')
    full = recording_topics(('134', '7'), stack, 'full')

    assert '/macortex_bridge/waverover_134/pose' in core
    assert '/waverover_swarm/predicted_path/waverover_7' in core
    assert '/waverover_134/waypoints' in core
    assert '/waverover_swarm/controller_telemetry' in core
    assert '/waverover_134/cmd_vel' in core
    assert '/waverover_134/scan' not in core
    assert '/waverover_134/scan' in full
    assert '/waverover_134/imu/data_raw' in full
    assert set(core) < set(full)
    overrides = qos_overrides(('134',), stack)
    assert overrides['/macortex_bridge/waverover_134/pose']['reliability'] == 'best_effort'
    assert overrides['/waverover_swarm/synthetic/metadata']['durability'] == 'transient_local'


def test_run_directories_are_safe_unique_and_never_overwritten(tmp_path):
    now = datetime(2026, 7, 18, 12, 34, 56, tzinfo=timezone.utc)
    first_id, first = create_run_directory(
        tmp_path, 'mpc_distributed', 'synthetic', 42, now
    )
    second_id, second = create_run_directory(
        tmp_path, 'mpc_distributed', 'synthetic', 42, now
    )

    assert first != second
    assert first_id != second_id
    assert re.fullmatch(r'[A-Za-z0-9_-]+', first_id)
    assert ':' not in str(first)
    assert (first / 'analysis').is_dir()


def test_manifest_updates_atomically(tmp_path):
    manifest = tmp_path / 'manifest.yaml'
    atomic_write_yaml(manifest, {'state': 'starting', 'schema_version': 1})
    atomic_write_yaml(manifest, {'state': 'completed', 'schema_version': 1})
    assert yaml.safe_load(manifest.read_text(encoding='utf-8')) == {
        'state': 'completed', 'schema_version': 1
    }
    assert list(tmp_path.glob('.*.tmp-*')) == []


def test_runner_uses_argument_lists_and_preserves_configured_dry_run():
    source = (
        Path(__file__).parents[1]
        / 'waverover_swarm_controller' / 'run_experiment.py'
    ).read_text(encoding='utf-8')
    assert 'shell=True' not in source
    assert "str(config.safety.dry_run).lower()" in source
    assert "'waypoint_dispatch': asdict(config.waypoint_dispatch)" in source
    assert "dry_run_override=True" not in source
    assert "['dry_run'] = True" not in source
    assert '/waverover_swarm/arm' not in source
    assert "'BEGIN'" in source and "'END'" in source


@pytest.mark.parametrize('dry_run', [True, False])
def test_resolved_run_config_preserves_safety_dry_run(
    tmp_path, dry_run
):
    source = tmp_path / 'source.yaml'
    destination = tmp_path / 'resolved.yaml'
    source.write_text(yaml.safe_dump({
        'controller': {'algorithm': 'heuristic'},
        'safety': {'dry_run': dry_run},
        'synthetic_mcs': {'seed': 1},
        'targets_file': '/original/targets.yaml',
    }), encoding='utf-8')

    _resolved_run_config(source, destination, 'convex', 42)

    resolved = yaml.safe_load(destination.read_text(encoding='utf-8'))
    assert resolved['safety']['dry_run'] is dry_run
    assert resolved['controller']['algorithm'] == 'heuristic'
    assert resolved['synthetic_mcs']['seed'] == 42
    assert resolved['targets_file'] == 'targets.yaml'


def test_cli_algorithm_override_validation_and_selection_metadata():
    path = Path(__file__).parents[1] / 'config' / 'experiment.yaml'
    configured, effective, metadata = resolve_algorithm_selection(path)
    assert configured.controller.algorithm == 'heuristic'
    assert effective.controller.algorithm == 'heuristic'
    assert metadata == {
        'configured_algorithm': 'heuristic',
        'effective_algorithm': 'heuristic',
        'algorithm_source': 'config',
    }

    configured, effective, metadata = resolve_algorithm_selection(path, 'convex')
    assert configured.controller.algorithm == 'heuristic'
    assert effective.controller.algorithm == 'convex'
    assert metadata['algorithm_source'] == 'cli'
    assert parse_arguments(['--config', str(path)]).algorithm is None
    with pytest.raises(SystemExit):
        parse_arguments(['--config', str(path), '--algorithm', 'mpc'])


def test_known_paper_metrics_and_yaw_wrap():
    config = example_config()
    sample = {
        'robots': {
            '131': {'position': [0.0, 0.0], 'heading_rad': 0.0},
            '132': {'position': [1.0, 0.0], 'heading_rad': 0.0},
        },
        'station': {'id': 'station_0', 'position': [-1.0, 0.0]},
        'targets': {
            'main': {'position': [2.0, 0.0], 'weight': 2.0, 'is_main': True}
        },
        'selected_edges': [['station_0', '131'], ['131', '132']],
        'target_assignments': {'132': 'main'},
    }
    communication, target, total, exact = mission_cost(sample, 1.5)
    assert (communication, target, total, exact) == pytest.approx((3.0, 2.0, 5.0, True))
    exact_main, proxy_main = main_target_distances(sample)
    assert exact_main == pytest.approx(1.0)
    assert proxy_main == pytest.approx(1.0)
    separation, pair = separation_metrics(sample)
    assert separation == pytest.approx(1.0)
    assert pair == ['131', '132']
    graph = graph_metrics(sample, config)
    assert graph['connected_components'] == 1
    midpoint = interpolate_angle(math_radians(179.0), math_radians(-179.0), 0.5)
    assert abs(abs(midpoint) - 3.141592653589793) < 1e-6


def math_radians(value):
    return value * 3.141592653589793 / 180.0


def test_irregular_pose_interpolation_respects_gap_and_yaw_wrap():
    series = [
        (0.0, 0.0, 0.0, math_radians(179.0)),
        (0.3, 0.3, 0.6, math_radians(-179.0)),
    ]
    x, y, yaw = interpolate_pose(series, 0.15, maximum_gap=0.5)
    assert (x, y) == pytest.approx((0.15, 0.3))
    assert abs(abs(yaw) - 3.141592653589793) < 1e-6
    assert interpolate_pose(series, 0.15, maximum_gap=0.1) in (
        series[0][1:], series[1][1:]
    )
