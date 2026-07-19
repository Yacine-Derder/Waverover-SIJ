"""Aggregate compatible completed WaveRover experiment analyses."""

import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
import yaml

from .analyze_run import analyze
from .experiment_recording import utc_timestamp


SCALAR_PATHS = {
    'cumulative_J': ('mission_cost', 'cumulative_integral'),
    'd_main_mean': ('main_target_distance', 'minimum_any_rover_proxy', 'mean'),
    'lambda_2_mean': ('connectivity', 'binary_lambda_2', 'mean'),
    'computation_time_mean': ('computation', 'duration_sec', 'mean'),
    'connectivity_outages': ('connectivity', 'outage_count'),
    'minimum_separation': ('separation', 'current_m', 'minimum'),
    'deadline_misses': ('computation', 'deadline_misses'),
    'path_length_mean': ('motion', 'mean_path_length_m'),
    'tracking_error_mean': ('motion', 'setpoint_tracking_error_m', 'mean'),
}


def _nested(values, path):
    selected = values
    for key in path:
        if selected is None or key not in selected:
            return None
        selected = selected[key]
    return selected


def discover_runs(paths):
    manifests = []
    for value in paths:
        path = Path(value).expanduser().resolve()
        if path.is_file() and path.name == 'manifest.yaml':
            manifests.append(path)
        elif (path / 'manifest.yaml').exists():
            manifests.append(path / 'manifest.yaml')
        elif path.exists():
            manifests.extend(path.rglob('manifest.yaml'))
    return tuple(sorted(set(manifests)))


def compatibility_key(manifest):
    targets = tuple(sorted(
        (target['target_id'], target['x'], target['y'], target['weight'])
        for target in manifest.get('targets', [])
    ))
    communication = manifest.get('communication', {})
    return (
        manifest.get('algorithm'),
        manifest.get('synthetic_mode'),
        len(manifest.get('robot_ids', [])),
        len(targets),
        targets,
        communication.get('ideal_range_m'),
        communication.get('maximum_range_m'),
        manifest.get('requested_duration_sec'),
    )


def compare(paths, output_directory=None):
    groups = defaultdict(list)
    skipped = []
    for manifest_path in discover_runs(paths):
        manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
        if manifest.get('state') != 'completed':
            skipped.append({
                'run_id': manifest.get('run_id'),
                'reason': 'state=' + str(manifest.get('state')),
            })
            continue
        run_directory = manifest_path.parent
        summary_path = run_directory / 'analysis' / 'summary.json'
        if not summary_path.exists():
            analyze(run_directory)
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
        groups[compatibility_key(manifest)].append((manifest, summary))
    if output_directory is None:
        base = Path(paths[0]).expanduser().resolve()
        if base.is_file():
            base = base.parent
        output_directory = base / ('comparison_' + utc_timestamp())
    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=False)
    output = {'schema_version': 1, 'groups': [], 'skipped_runs': skipped}
    for index, (key, runs) in enumerate(sorted(groups.items(), key=str), 1):
        aggregate = {}
        for label, path in SCALAR_PATHS.items():
            values = [
                _nested(summary, path) for _manifest, summary in runs
            ]
            values = [float(value) for value in values if value is not None]
            aggregate[label] = (
                {
                    'mean': float(np.mean(values)),
                    'stddev': float(np.std(values)),
                    'minimum': float(np.min(values)),
                    'maximum': float(np.max(values)),
                    'values': values,
                } if values else None
            )
        output['groups'].append({
            'group_id': index,
            'algorithm': key[0],
            'synthetic_mode': key[1],
            'rover_count': key[2],
            'target_count': key[3],
            'requested_duration_sec': key[7],
            'seeds': [manifest.get('synthetic_seed') for manifest, _ in runs],
            'run_ids': [manifest.get('run_id') for manifest, _ in runs],
            'metrics': aggregate,
        })
    (output_directory / 'comparison.json').write_text(
        json.dumps(output, indent=2, sort_keys=True), encoding='utf-8'
    )
    with (output_directory / 'comparison.yaml').open('w', encoding='utf-8') as stream:
        yaml.safe_dump(output, stream, sort_keys=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    labels = list(SCALAR_PATHS)
    figure, axes = plt.subplots(3, 3, figsize=(15, 12))
    for axis, label in zip(axes.flat, labels):
        group_labels = []
        means = []
        deviations = []
        for group in output['groups']:
            metric = group['metrics'][label]
            if metric is not None:
                group_labels.append('%s/g%d' % (
                    group['algorithm'], group['group_id']
                ))
                means.append(metric['mean'])
                deviations.append(metric['stddev'])
        if means:
            axis.bar(group_labels, means, yerr=deviations)
            axis.tick_params(axis='x', labelrotation=30)
        axis.set_title(label)
        axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(output_directory / 'comparison_metrics.png', dpi=150)
    plt.close(figure)
    return output_directory


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('paths', nargs='+')
    parser.add_argument('--output')
    arguments = parser.parse_args(args)
    print(compare(arguments.paths, arguments.output))


if __name__ == '__main__':
    main()
