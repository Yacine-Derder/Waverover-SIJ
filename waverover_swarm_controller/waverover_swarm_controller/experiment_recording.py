"""Pure run-directory, topic-profile, and manifest helpers."""

from datetime import datetime, timezone
import os
from pathlib import Path
import secrets

import yaml

from waverover.stack_config import mcs_pose_topic, robot_namespace, robot_topic


MANIFEST_SCHEMA_VERSION = 2
RUN_EVENT_SCHEMA_VERSION = 1


def utc_timestamp(now=None):
    value = now or datetime.now(timezone.utc)
    return value.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def default_run_root():
    xdg_data = os.environ.get('XDG_DATA_HOME')
    base = Path(xdg_data).expanduser() if xdg_data else Path.home() / '.local/share'
    return base / 'waverover' / 'runs'


def create_run_directory(root, algorithm, pose_source, seed, now=None):
    timestamp = utc_timestamp(now)
    date_directory = timestamp[:4] + '-' + timestamp[4:6] + '-' + timestamp[6:8]
    seed_label = str(seed) if seed is not None else 'random'
    for _attempt in range(100):
        suffix = secrets.token_hex(3)
        run_id = '_'.join((
            timestamp,
            str(algorithm),
            str(pose_source),
            seed_label,
            suffix,
        ))
        run_directory = Path(root) / date_directory / run_id
        try:
            run_directory.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            continue
        for name in ('config', 'bag', 'logs', 'analysis'):
            (run_directory / name).mkdir()
        return run_id, run_directory
    raise RuntimeError('Could not allocate a unique experiment run directory.')


def recording_topics(robot_ids, stack_config, profile='core'):
    topics = {
        '/waverover_swarm/run_event',
        '/waverover_swarm/synthetic/metadata',
        '/waverover_swarm/controller_telemetry',
        '/waverover_swarm/target_state',
        '/waverover_swarm/diagnostics',
        '/waverover_swarm/markers',
        '/parameter_events',
        '/rosout',
        '/tf',
        '/tf_static',
    }
    for robot_id in sorted(str(value) for value in robot_ids):
        namespace = robot_namespace(stack_config, robot_id)
        topics.update({
            mcs_pose_topic(stack_config, robot_id),
            '/waverover_swarm/synthetic/ground_truth/' + namespace,
            '/waverover_swarm/synthetic/motion/' + namespace,
            '/waverover_swarm/predicted_path/' + namespace,
            robot_topic(stack_config, 'waypoints', robot_id),
            robot_topic(stack_config, 'waypoint_reached', robot_id),
            robot_topic(stack_config, 'end_trial', robot_id),
            robot_topic(stack_config, 'cmd_vel', robot_id),
        })
        if profile == 'full':
            topics.update({
                robot_topic(stack_config, 'imu', robot_id),
                robot_topic(stack_config, 'odom', robot_id),
                robot_topic(stack_config, 'scan', robot_id),
                robot_topic(stack_config, 'map', robot_id),
                robot_topic(stack_config, 'map_metadata', robot_id),
            })
    return tuple(sorted(topics))


def qos_overrides(robot_ids, stack_config):
    values = {
        '/waverover_swarm/target_state': {
            'history': 'keep_last', 'depth': 10,
            'reliability': 'reliable', 'durability': 'transient_local',
        },
        '/waverover_swarm/synthetic/metadata': {
            'history': 'keep_last',
            'depth': 1,
            'reliability': 'reliable',
            'durability': 'transient_local',
        },
        '/waverover_swarm/run_event': {
            'history': 'keep_last',
            'depth': 10,
            'reliability': 'reliable',
            'durability': 'transient_local',
        },
        '/tf_static': {
            'history': 'keep_all',
            'reliability': 'reliable',
            'durability': 'transient_local',
        },
    }
    for robot_id in sorted(str(value) for value in robot_ids):
        values[mcs_pose_topic(stack_config, robot_id)] = {
            'history': 'keep_last',
            'depth': 20,
            'reliability': 'best_effort',
            'durability': 'volatile',
        }
    return values


def atomic_write_yaml(path, values):
    path = Path(path)
    temporary = path.with_name('.' + path.name + '.tmp-' + secrets.token_hex(4))
    with temporary.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(values, stream, sort_keys=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def write_qos_overrides(path, values):
    atomic_write_yaml(path, values)
