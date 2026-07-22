import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import String

from waverover_swarm_controller.analyze_run import analyze
from waverover_swarm_controller.compare_runs import compare
from waverover_swarm_controller.replay_run import replay
import yaml


def _string(value):
    message = String()
    message.data = json.dumps(value, sort_keys=True)
    return message


def _pose(timestamp_sec, x, y, yaw_z=0.0, yaw_w=1.0):
    message = PoseStamped()
    message.header.frame_id = 'robotics_lab'
    message.header.stamp.sec = int(timestamp_sec)
    message.header.stamp.nanosec = int((timestamp_sec % 1.0) * 1e9)
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = yaw_z
    message.pose.orientation.w = yaw_w
    return message


def _motion(timestamp_sec):
    message = TwistStamped()
    message.header.stamp.sec = int(timestamp_sec)
    message.twist.linear.x = 0.3
    message.twist.angular.z = 0.0
    return message


def _make_run(tmp_path, state='completed'):
    package = Path(__file__).parents[1]
    run = tmp_path / 'run'
    (run / 'config').mkdir(parents=True)
    (run / 'bag').mkdir()
    (run / 'analysis').mkdir()
    config = yaml.safe_load(
        (package / 'config' / 'experiment.yaml').read_text(encoding='utf-8')
    )
    config['robot_ids'] = ['134']
    config['targets_file'] = 'targets.yaml'
    (run / 'config' / 'experiment.yaml').write_text(
        yaml.safe_dump(config), encoding='utf-8'
    )
    (run / 'config' / 'targets.yaml').write_bytes(
        (package / 'config' / 'targets.yaml').read_bytes()
    )
    manifest = {
        'schema_version': 1,
        'run_id': 'fixture',
        'state': state,
        'algorithm': 'mpc_distributed',
        'configured_algorithm': 'heuristic',
        'effective_algorithm': 'mpc_distributed',
        'algorithm_source': 'cli',
        'synthetic_mode': 'preset',
        'synthetic_seed': 42,
        'robot_ids': ['134'],
        'targets': [
            {'target_id': 'target_main', 'x': 2.5, 'y': 0.0, 'weight': 10.0}
        ],
        'communication': {'ideal_range_m': 1.5, 'maximum_range_m': 2.0},
        'resolved_config': 'config/experiment.yaml',
        'resolved_targets': 'config/targets.yaml',
        'bag_storage_id': 'sqlite3',
    }
    (run / 'manifest.yaml').write_text(
        yaml.safe_dump(manifest), encoding='utf-8'
    )
    bag = run / 'bag' / 'recording'
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topics = {
        '/waverover_swarm/run_event': 'std_msgs/msg/String',
        '/waverover_swarm/controller_telemetry': 'std_msgs/msg/String',
        '/macortex_bridge/waverover_134/pose': 'geometry_msgs/msg/PoseStamped',
        '/waverover_swarm/synthetic/ground_truth/waverover_134': (
            'geometry_msgs/msg/PoseStamped'
        ),
        '/waverover_swarm/synthetic/motion/waverover_134': (
            'geometry_msgs/msg/TwistStamped'
        ),
    }
    for identifier, (topic, type_name) in enumerate(topics.items(), 1):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=identifier,
            name=topic,
            type=type_name,
            serialization_format='cdr',
        ))
    begin = _string({'schema_version': 1, 'run_id': 'fixture', 'event': 'BEGIN'})
    end = _string({'schema_version': 1, 'run_id': 'fixture', 'event': 'END'})
    writer.write('/waverover_swarm/run_event', serialize_message(begin), 1_000_000_000)
    for index, timestamp in enumerate((1.5, 2.5, 3.5)):
        x = 0.1 * index
        telemetry = _string({
            'schema_version': 1,
            'algorithm': 'mpc_distributed',
            'result_state': 'valid',
            'solver_status': 'optimal',
            'solve_duration_sec': 0.1 + 0.01 * index,
            'robots': {'134': {'position': [x, 0.0], 'heading_rad': 0.0}},
            'station': {'id': 'station_0', 'position': [0.0, 0.0]},
            'targets': {
                'target_main': {
                    'position': [2.5, 0.0], 'weight': 10.0, 'is_main': True
                }
            },
            'setpoints': {'134': [x + 0.1, 0.0]},
            'active_waypoints': {'134': None},
            'pending_waypoints': {'134': [x + 0.1, 0.0]},
            'predicted_paths': {'134': [[x, 0.0], [x + 0.1, 0.0]]},
            'selected_edges': [['station_0', '134']],
            'target_assignments': {'134': 'target_main'},
            'predicted_minimum_separation': {
                'distance_m': None, 'pair': None, 'step': None
            },
            'snapshot_skew_sec': 0.0,
            'stop_reason': '',
        })
        timestamp_ns = int(timestamp * 1e9)
        writer.write(
            '/waverover_swarm/controller_telemetry',
            serialize_message(telemetry),
            timestamp_ns,
        )
        pose = _pose(timestamp, x, 0.0)
        writer.write(
            '/macortex_bridge/waverover_134/pose',
            serialize_message(pose),
            timestamp_ns,
        )
        writer.write(
            '/waverover_swarm/synthetic/ground_truth/waverover_134',
            serialize_message(pose),
            timestamp_ns,
        )
        writer.write(
            '/waverover_swarm/synthetic/motion/waverover_134',
            serialize_message(_motion(timestamp)),
            timestamp_ns,
        )
    writer.write('/waverover_swarm/run_event', serialize_message(end), 4_000_000_000)
    del writer
    return run


def test_programmatic_bag_analysis_and_headless_replay(tmp_path):
    run = _make_run(tmp_path)
    analysis_directory = analyze(run)

    expected = {
        'summary.yaml', 'summary.json', 'timeseries.csv', 'events.csv',
        'metrics_over_time.png', 'metric_distributions.png',
        'trajectories.png', 'report.md',
    }
    assert expected <= {path.name for path in analysis_directory.iterdir()}
    summary = json.loads(
        (analysis_directory / 'summary.json').read_text(encoding='utf-8')
    )
    assert summary['mission_cost']['exact_assignments_available']
    assert summary['computation']['duration_sec']['mean'] == 0.11
    assert summary['motion']['path_length_m_by_rover']['134'] == 0.2
    assert summary['warnings'] == []

    output = run / 'analysis' / 'frame.png'
    replay(run, selected_time=1.5, output=output, no_show=True)
    assert output.exists() and output.stat().st_size > 1000


def test_comparison_separates_incompatible_configs(tmp_path):
    first = _make_run(tmp_path / 'first')
    second = _make_run(tmp_path / 'second')
    analyze(first)
    analyze(second)
    manifest_path = second / 'manifest.yaml'
    manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    manifest['communication']['maximum_range_m'] = 3.0
    manifest_path.write_text(yaml.safe_dump(manifest), encoding='utf-8')

    output = compare([first, second], tmp_path / 'comparison')
    comparison = json.loads(
        (output / 'comparison.json').read_text(encoding='utf-8')
    )
    assert len(comparison['groups']) == 2


def test_interrupted_run_remains_discoverable_but_comparison_skips_it(tmp_path):
    run = _make_run(tmp_path, state='interrupted')
    output = compare([run], tmp_path / 'comparison')
    comparison = json.loads(
        (output / 'comparison.json').read_text(encoding='utf-8')
    )
    assert comparison['groups'] == []
    assert comparison['skipped_runs'][0]['reason'] == 'state=interrupted'
