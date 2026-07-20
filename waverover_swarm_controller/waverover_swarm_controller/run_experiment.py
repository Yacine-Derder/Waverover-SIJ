"""Supervise a configured coordinator, optional synthetic MCS, and rosbag2."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import secrets
import signal
import socket
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import yaml

from waverover.stack_config import load_stack_config

from .config import load_experiment
from .experiment_recording import (
    MANIFEST_SCHEMA_VERSION,
    RUN_EVENT_SCHEMA_VERSION,
    atomic_write_yaml,
    create_run_directory,
    default_run_root,
    qos_overrides,
    recording_topics,
    utc_timestamp,
    write_qos_overrides,
)
from .synthetic_motion import derive_rover_seed


class RunEventPublisher(Node):
    def __init__(self, config, stack_config):
        super().__init__('waverover_run_event')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String, '/waverover_swarm/run_event', qos
        )
        self.synthetic_state = None
        self.synthetic_error = ''
        self.synthetic_subscription = self.create_subscription(
            String,
            '/waverover_swarm/synthetic/metadata',
            self._synthetic_metadata,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # Advertise command/TF topics without publishing from this node so bags
        # have their schemas even when a dry-run records zero messages.
        from waverover.stack_config import robot_topic
        self.schema_publishers = [
            self.create_publisher(
                Twist, robot_topic(stack_config, 'cmd_vel', robot_id), 1
            )
            for robot_id in config.robot_ids
        ]
        self.schema_publishers.extend((
            self.create_publisher(TFMessage, '/tf', 1),
            self.create_publisher(TFMessage, '/tf_static', qos),
        ))

    def _synthetic_metadata(self, message):
        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, AttributeError):
            return
        self.synthetic_state = value.get('state')
        self.synthetic_error = str(value.get('error', ''))

    def publish_event(self, run_id, event, detail=''):
        message = String()
        message.data = json.dumps({
            'schema_version': RUN_EVENT_SCHEMA_VERSION,
            'run_id': run_id,
            'event': event,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'detail': str(detail),
        }, sort_keys=True, separators=(',', ':'))
        self.publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.15)


def _git_information(repository):
    def command(*arguments):
        result = subprocess.run(
            ['git', '-C', str(repository), *arguments],
            check=False,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip() if result.returncode == 0 else None

    root = command('rev-parse', '--show-toplevel')
    if root is None:
        return None
    status = command('status', '--porcelain') or ''
    return {
        'repository': root,
        'branch': command('branch', '--show-current'),
        'commit': command('rev-parse', 'HEAD'),
        'dirty': bool(status),
    }


def _stop_child(child, timeout=8.0):
    if child is None or child.poll() is not None:
        return child.returncode if child is not None else None
    child.send_signal(signal.SIGINT)
    try:
        return child.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        child.terminate()
    try:
        return child.wait(timeout=4.0)
    except subprocess.TimeoutExpired:
        child.kill()
        return child.wait(timeout=2.0)


def _start_child(command, log_path):
    stream = Path(log_path).open('w', encoding='utf-8')
    process = subprocess.Popen(
        command,
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=False,
    )
    process._waverover_log_stream = stream
    return process


def _close_child_log(child):
    stream = getattr(child, '_waverover_log_stream', None)
    if stream is not None:
        stream.close()


def _resolved_run_config(config_path, destination, algorithm, seed):
    source = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    source.setdefault('controller', {})['algorithm'] = algorithm
    source.setdefault('synthetic_mcs', {})['seed'] = int(seed)
    source['targets_file'] = 'targets.yaml'
    with Path(destination).open('w', encoding='utf-8') as stream:
        yaml.safe_dump(source, stream, sort_keys=False)


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(
        description='Record a reproducible PC-only WaveRover experiment.'
    )
    parser.add_argument('--config', required=True)
    parser.add_argument('--algorithm')
    parser.add_argument('--duration-sec', type=float)
    parser.add_argument('--output-root')
    parser.add_argument('--profile', choices=('core', 'full'))
    seed_group = parser.add_mutually_exclusive_group()
    seed_group.add_argument('--seed', type=int)
    seed_group.add_argument('--fresh-seed', action='store_true')
    parser.add_argument('--no-synthetic', action='store_true')
    return parser.parse_args(arguments)


def main(args=None):
    arguments = parse_arguments(args)
    original_config = load_experiment(arguments.config)
    algorithm = arguments.algorithm or original_config.controller.algorithm
    requested_seed = (
        None if arguments.fresh_seed else (
            arguments.seed
            if arguments.seed is not None else original_config.synthetic_mcs.seed
        )
    )
    actual_seed = (
        secrets.randbits(63) if requested_seed is None else int(requested_seed)
    )
    root = (
        Path(arguments.output_root).expanduser()
        if arguments.output_root else (
            Path(original_config.recording.root_directory).expanduser()
            if original_config.recording.root_directory else default_run_root()
        )
    )
    profile = arguments.profile or original_config.recording.profile
    pose_source = original_config.recording.pose_source
    run_id, run_directory = create_run_directory(
        root, algorithm, pose_source, actual_seed
    )
    resolved_config_path = run_directory / 'config' / 'experiment.yaml'
    targets_copy = run_directory / 'config' / 'targets.yaml'
    targets_copy.write_bytes(Path(original_config.targets_path).read_bytes())
    _resolved_run_config(
        original_config.source_path,
        resolved_config_path,
        algorithm,
        actual_seed,
    )
    config = load_experiment(resolved_config_path)
    duration = (
        arguments.duration_sec
        if arguments.duration_sec is not None
        else config.synthetic_mcs.duration_sec
    )
    if duration <= 0.0:
        raise ValueError('duration-sec must be positive.')
    stack_config = load_stack_config(require_identity=False)
    topics = recording_topics(config.robot_ids, stack_config, profile)
    qos_path = run_directory / 'config' / 'rosbag_qos_overrides.yaml'
    write_qos_overrides(
        qos_path, qos_overrides(config.robot_ids, stack_config)
    )
    bag_directory = run_directory / 'bag' / 'recording'
    bag_command = [
        'ros2', 'bag', 'record',
        '--storage', config.recording.storage_id,
        '--output', str(bag_directory),
        '--qos-profile-overrides-path', str(qos_path),
        '--topics',
        *topics,
    ]
    synthetic_command = [
        'ros2', 'launch', 'waverover_swarm_controller',
        'synthetic_mcs.launch.py',
        'config_file:=' + str(resolved_config_path),
    ]
    coordinator_command = [
        'ros2', 'launch', 'waverover_swarm_controller',
        'swarm_controller.launch.py',
        'config_file:=' + str(resolved_config_path),
        'algorithm:=' + algorithm,
        'dry_run:=' + str(config.safety.dry_run).lower(),
    ]
    git = _git_information(original_config.source_path.parent)
    try:
        package_version = importlib.metadata.version('waverover_swarm_controller')
    except importlib.metadata.PackageNotFoundError:
        package_version = None
    started = datetime.now(timezone.utc)
    manifest = {
        'schema_version': MANIFEST_SCHEMA_VERSION,
        'run_id': run_id,
        'state': 'starting',
        'start_timestamp': started.isoformat(),
        'end_timestamp': None,
        'duration_sec': None,
        'experiment_config_path': str(original_config.source_path),
        'resolved_config': 'config/experiment.yaml',
        'resolved_targets': 'config/targets.yaml',
        'algorithm': algorithm,
        'dry_run': config.safety.dry_run,
        'pose_source': pose_source,
        'synthetic_mode': config.synthetic_mcs.mode,
        'synthetic_seed': actual_seed,
        'synthetic_formation_coupling': (
            config.synthetic_mcs.formation_coupling
        ),
        'synthetic_connectivity_policy': (
            config.synthetic_mcs.connectivity_policy
        ),
        'synthetic_initial_radius_m': config.synthetic_mcs.initial_radius_m,
        'synthetic_derived_rover_seeds': {
            robot_id: derive_rover_seed(actual_seed, robot_id)
            for robot_id in sorted(config.robot_ids)
        },
        'requested_duration_sec': duration,
        'robot_ids': list(config.robot_ids),
        'station': asdict(config.station),
        'targets': [asdict(target) for target in config.targets],
        'communication': asdict(config.communication),
        'waypoint_dispatch': asdict(config.waypoint_dispatch),
        'safety': asdict(config.safety),
        'host': socket.gethostname(),
        'ros_distribution': os.environ.get('ROS_DISTRO'),
        'python_version': platform.python_version(),
        'git': git,
        'package_version': package_version,
        'commands': {
            'bag': bag_command,
            'synthetic_mcs': (
                synthetic_command if not arguments.no_synthetic
                and config.recording.start_synthetic else None
            ),
            'coordinator': coordinator_command,
        },
        'bag_storage_id': config.recording.storage_id,
        'recording_profile': profile,
        'requested_topics': list(topics),
        'recorded_topics': [],
        'bag_size_bytes': None,
        'child_exit_codes': {},
        'failure_reason': '',
    }
    manifest_path = run_directory / 'manifest.yaml'
    atomic_write_yaml(manifest_path, manifest)
    if git and git['dirty']:
        patch = subprocess.run(
            ['git', '-C', git['repository'], 'diff', '--binary'],
            check=False,
            capture_output=True,
        )
        (run_directory / 'working_tree.patch').write_bytes(patch.stdout)

    interrupted = False
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal interrupted, stop_requested
        interrupted = True
        stop_requested = True

    previous_handlers = {
        signum: signal.getsignal(signum)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    for signum in previous_handlers:
        signal.signal(signum, request_stop)

    bag = synthetic = coordinator = None
    event_node = None
    failure = ''
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        event_node = RunEventPublisher(config, stack_config)
        bag = _start_child(bag_command, run_directory / 'logs' / 'rosbag.log')
        time.sleep(1.0)
        if bag.poll() is not None:
            raise RuntimeError('rosbag exited before experiment startup.')
        event_node.publish_event(run_id, 'BEGIN')
        if not arguments.no_synthetic and config.recording.start_synthetic:
            synthetic = _start_child(
                synthetic_command, run_directory / 'logs' / 'synthetic_mcs.log'
            )
        coordinator = _start_child(
            coordinator_command, run_directory / 'logs' / 'coordinator.log'
        )
        manifest['state'] = 'running'
        atomic_write_yaml(manifest_path, manifest)
        await_synthetic_completion = (
            synthetic is not None and arguments.duration_sec is None
        )
        deadline = time.monotonic() + duration + (
            max(15.0, 0.25 * duration) if await_synthetic_completion else 0.0
        )
        while not stop_requested and time.monotonic() < deadline:
            if bag.poll() is not None:
                raise RuntimeError('rosbag exited unexpectedly.')
            if coordinator.poll() is not None:
                raise RuntimeError('coordinator exited unexpectedly.')
            if synthetic is not None and synthetic.poll() is not None:
                raise RuntimeError('synthetic_mcs exited unexpectedly.')
            rclpy.spin_once(event_node, timeout_sec=0.1)
            if event_node.synthetic_state == 'failed':
                raise RuntimeError(
                    'synthetic_mcs failed: %s' % event_node.synthetic_error
                )
            if (
                await_synthetic_completion
                and event_node.synthetic_state == 'completed'
            ):
                break
        if (
            await_synthetic_completion
            and not stop_requested
            and event_node.synthetic_state != 'completed'
        ):
            raise RuntimeError(
                'synthetic_mcs did not complete within the bounded grace period.'
            )
        event_node.publish_event(run_id, 'STOP_REQUESTED')
    except Exception as error:
        failure = str(error)
        if event_node is not None:
            event_node.publish_event(run_id, 'ERROR', failure)
    finally:
        manifest['child_exit_codes']['coordinator'] = _stop_child(coordinator)
        manifest['child_exit_codes']['synthetic_mcs'] = _stop_child(synthetic)
        time.sleep(0.5)
        if event_node is not None:
            event_node.publish_event(run_id, 'END', failure)
        time.sleep(0.5)
        manifest['child_exit_codes']['bag'] = _stop_child(bag)
        for child in (coordinator, synthetic, bag):
            if child is not None:
                _close_child_log(child)
        if event_node is not None:
            event_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        ended = datetime.now(timezone.utc)
        metadata_path = bag_directory / 'metadata.yaml'
        if metadata_path.exists():
            bag_metadata = yaml.safe_load(
                metadata_path.read_text(encoding='utf-8')
            ).get('rosbag2_bagfile_information', {})
            manifest['recorded_topics'] = sorted(
                value['topic_metadata']['name']
                for value in bag_metadata.get('topics_with_message_count', [])
            )
        manifest['bag_size_bytes'] = sum(
            path.stat().st_size
            for path in bag_directory.rglob('*')
            if path.is_file()
        ) if bag_directory.exists() else 0
        manifest['end_timestamp'] = ended.isoformat()
        manifest['duration_sec'] = (ended - started).total_seconds()
        manifest['failure_reason'] = failure
        manifest['state'] = (
            'failed' if failure else ('interrupted' if interrupted else 'completed')
        )
        atomic_write_yaml(manifest_path, manifest)
    print(run_directory)
    return 1 if failure else 0


if __name__ == '__main__':
    sys.exit(main())
