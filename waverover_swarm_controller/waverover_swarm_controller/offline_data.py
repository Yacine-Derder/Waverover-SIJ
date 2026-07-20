"""Selective rosbag2 loading for experiment analysis and replay."""

from collections import defaultdict
import json
from pathlib import Path

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml

from .config import load_experiment


def message_timestamp_sec(message, bag_timestamp_ns):
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is not None:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if value > 0.0:
            return value
    return float(bag_timestamp_ns) * 1e-9


def locate_bag(run_directory):
    bag_root = Path(run_directory) / 'bag'
    candidates = sorted(bag_root.rglob('metadata.yaml'))
    if not candidates:
        raise FileNotFoundError('No rosbag2 metadata.yaml found under %s.' % bag_root)
    return candidates[0].parent


def load_run_data(run_directory):
    run_directory = Path(run_directory).expanduser().resolve()
    manifest_path = run_directory / 'manifest.yaml'
    if not manifest_path.exists():
        raise FileNotFoundError('Missing run manifest %s.' % manifest_path)
    manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    config_path = run_directory / manifest.get(
        'resolved_config', 'config/experiment.yaml'
    )
    config = load_experiment(config_path)
    bag_directory = locate_bag(run_directory)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_directory),
            storage_id=manifest.get('bag_storage_id', 'sqlite3'),
        ),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }
    interesting = {
        '/waverover_swarm/controller_telemetry',
        '/waverover_swarm/synthetic/metadata',
        '/waverover_swarm/run_event',
        '/waverover_swarm/target_state',
    }
    for robot_id in config.robot_ids:
        interesting.update({
            '/macortex_bridge/waverover_%s/pose' % robot_id,
            '/waverover_swarm/synthetic/ground_truth/waverover_%s' % robot_id,
            '/waverover_swarm/synthetic/motion/waverover_%s' % robot_id,
            '/waverover_%s/waypoints' % robot_id,
            '/waverover_%s/waypoint_reached' % robot_id,
            '/waverover_%s/cmd_vel' % robot_id,
        })
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in interesting
    }
    samples = defaultdict(list)
    first_bag_time = None
    last_bag_time = None
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        first_bag_time = timestamp_ns if first_bag_time is None else first_bag_time
        last_bag_time = timestamp_ns
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        timestamp_sec = (
            float(timestamp_ns) * 1e-9
            if topic.endswith('/waypoints')
            or topic.endswith('/waypoint_reached')
            else message_timestamp_sec(message, timestamp_ns)
        )
        samples[topic].append({
            'bag_timestamp_sec': float(timestamp_ns) * 1e-9,
            'timestamp_sec': timestamp_sec,
            'message': message,
        })
    telemetry = []
    for sample in samples['/waverover_swarm/controller_telemetry']:
        try:
            value = json.loads(sample['message'].data)
        except (json.JSONDecodeError, AttributeError):
            continue
        value['_timestamp_sec'] = sample['timestamp_sec']
        telemetry.append(value)
    events = []
    for sample in samples['/waverover_swarm/run_event']:
        try:
            value = json.loads(sample['message'].data)
        except (json.JSONDecodeError, AttributeError):
            continue
        value['_timestamp_sec'] = sample['timestamp_sec']
        events.append(value)
    metadata = []
    for sample in samples['/waverover_swarm/synthetic/metadata']:
        try:
            value = json.loads(sample['message'].data)
        except (json.JSONDecodeError, AttributeError):
            continue
        value['_timestamp_sec'] = sample['timestamp_sec']
        metadata.append(value)
    target_states = []
    for sample in samples['/waverover_swarm/target_state']:
        try:
            value = json.loads(sample['message'].data)
        except (json.JSONDecodeError, AttributeError):
            continue
        value['_timestamp_sec'] = sample['timestamp_sec']
        target_states.append(value)
    acknowledgements = [
        {
            'robot_id': topic.split('/')[1].removeprefix('waverover_'),
            'timestamp_sec': sample['timestamp_sec'],
            'frame_id': sample['message'].header.frame_id,
            'token': [
                sample['message'].header.stamp.sec,
                sample['message'].header.stamp.nanosec,
            ],
            'point': [sample['message'].point.x, sample['message'].point.y],
        }
        for topic, topic_samples in samples.items()
        if topic.endswith('/waypoint_reached')
        for sample in topic_samples
    ]
    start_candidates = [
        event['_timestamp_sec'] for event in events
        if event.get('event') == 'BEGIN'
    ]
    start_time = (
        min(start_candidates) if start_candidates
        else (float(first_bag_time) * 1e-9 if first_bag_time is not None else 0.0)
    )
    end_candidates = [
        event['_timestamp_sec'] for event in events
        if event.get('event') == 'END'
    ]
    end_time = (
        max(end_candidates) if end_candidates
        else (float(last_bag_time) * 1e-9 if last_bag_time is not None else start_time)
    )
    return {
        'run_directory': run_directory,
        'manifest': manifest,
        'config': config,
        'bag_directory': bag_directory,
        'topic_types': topic_types,
        'samples': samples,
        'telemetry': telemetry,
        'events': events,
        'synthetic_metadata': metadata,
        'target_states': target_states,
        'acknowledgement_events': acknowledgements,
        'start_time': start_time,
        'end_time': end_time,
        'used_run_events': bool(start_candidates and end_candidates),
    }
