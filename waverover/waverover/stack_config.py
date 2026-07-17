from copy import deepcopy
import math
import os
from pathlib import Path
import re
from string import Formatter

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
import yaml


SUPPORTED_CONTROL_MODES = ('twist', 'fixed_wing', 'manual_lr')
SUPPORTED_POSE_SOURCES = ('SLAM', 'MCS')
ROBOT_NAME_PATTERN = re.compile(r'[A-Za-z0-9_]+')


class StackConfigError(RuntimeError):
    """Raised when the canonical WaveRover configuration is invalid."""


def default_config_path():
    """Return the installed canonical stack configuration path."""
    try:
        share_directory = Path(get_package_share_directory('waverover'))
        installed_path = (
            share_directory / 'config' / 'robot_defaults.yaml'
        )
        if installed_path.is_file():
            return installed_path
    except PackageNotFoundError:
        installed_path = None

    # Supports source-tree tests and a stale overlay before rebuilding.
    source_path = Path(__file__).resolve().parents[1] / 'config' / (
        'robot_defaults.yaml'
    )
    if source_path.is_file():
        return source_path
    return installed_path or source_path


def default_identity_path(config_path=None):
    """Return the source-tree identity path used by onboard processes."""
    source_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'robot_identity.yaml'
    )
    if source_path.parent.is_dir():
        return source_path
    if config_path:
        return Path(config_path).resolve().with_name('robot_identity.yaml')
    return source_path


def _load_yaml_mapping(path, description):
    path = Path(path).expanduser()
    try:
        with path.open('r', encoding='utf-8') as stream:
            value = yaml.safe_load(stream)
    except OSError as error:
        raise StackConfigError(
            'Could not read WaveRover %s "%s": %s'
            % (description, path, error)
        ) from error
    except yaml.YAMLError as error:
        raise StackConfigError(
            'Could not parse WaveRover %s "%s": %s'
            % (description, path, error)
        ) from error
    if not isinstance(value, dict):
        raise StackConfigError(
            'WaveRover %s "%s" root must be a YAML mapping.'
            % (description, path)
        )
    return value


def load_robot_identity(identity_path=None, config_path=None):
    """Load the strict per-machine identity, honoring the environment."""
    environment_path = os.environ.get('WAVEROVER_IDENTITY_FILE')
    path = Path(environment_path) if environment_path else (
        Path(identity_path) if identity_path else default_identity_path(
            config_path
        )
    )
    identity = _load_yaml_mapping(path, 'robot identity')
    keys = set(identity)
    if keys != {'robot_name'}:
        missing = ' missing robot_name;' if 'robot_name' not in keys else ''
        unexpected = sorted(keys - {'robot_name'})
        extra = (
            ' unexpected keys: %s;' % ', '.join(unexpected)
            if unexpected else ''
        )
        raise StackConfigError(
            'Invalid WaveRover robot identity "%s":%s%s expected exactly '
            'one key: robot_name.' % (path, missing, extra)
        )
    try:
        robot_name = validate_robot_name(identity['robot_name'])
    except StackConfigError as error:
        raise StackConfigError(
            'Invalid WaveRover robot identity "%s": %s' % (path, error)
        ) from error
    return {'robot_name': robot_name}


def load_stack_config(
    require_identity=True,
    config_path=None,
    identity_path=None,
):
    """Load shared defaults and, for onboard use, per-machine identity."""
    path = Path(config_path) if config_path else default_config_path()
    config = _load_yaml_mapping(path, 'stack configuration')

    if config.get('schema_version') != 1:
        raise StackConfigError(
            'Unsupported WaveRover configuration schema_version: %r'
            % config.get('schema_version')
        )

    config['control_mode'] = normalize_control_mode(
        required(config, 'control_mode')
    )
    config['pose_source'] = normalize_pose_source(
        required(config, 'pose_source')
    )

    namespace_prefix = required(config, 'namespace_prefix')
    if not isinstance(namespace_prefix, str) or not namespace_prefix:
        raise StackConfigError('namespace_prefix must be a non-empty string.')

    for section in (
        'nodes',
        'topics',
        'frames',
        'mcs',
        'communication',
        'bridge',
        'lidar',
        'rf2o',
        'slam',
        'waypoint_controller',
        'manual_lr_ui',
        'waypoint_ui',
        'foxglove',
    ):
        if not isinstance(config.get(section), dict):
            raise StackConfigError(
                'WaveRover configuration section "%s" must be a mapping.'
                % section
            )

    for topic_key in (
        'cmd_vel',
        'waypoints',
        'end_trial',
        'scan',
        'imu',
        'odom',
        'manual_lr',
        'map',
        'map_metadata',
    ):
        topic_name = required(config, 'topics', topic_key)
        if not isinstance(topic_name, str) or not topic_name:
            raise StackConfigError(
                'topics.%s must be a non-empty relative topic name.'
                % topic_key
            )
        if topic_name.startswith('/'):
            raise StackConfigError(
                'topics.%s must stay relative so robot_name applies.'
                % topic_key
            )

    for frame_key in ('map', 'odom', 'base', 'lidar'):
        frame_name = required(config, 'frames', frame_key)
        if (
            not isinstance(frame_name, str)
            or not frame_name
            or frame_name.startswith('/')
        ):
            raise StackConfigError(
                'frames.%s must be a non-empty relative frame basename.'
                % frame_key
            )

    mcs_frame = required(config, 'mcs', 'frame')
    if (
        not isinstance(mcs_frame, str)
        or not mcs_frame
        or mcs_frame.startswith('/')
    ):
        raise StackConfigError(
            'mcs.frame must be a non-empty frame ID without a leading slash.'
        )
    mcs_timeout = required(config, 'mcs', 'pose_timeout_sec')
    if not isinstance(mcs_timeout, (int, float)) or mcs_timeout <= 0.0:
        raise StackConfigError('mcs.pose_timeout_sec must be positive.')
    mcs_qos_depth = required(config, 'mcs', 'qos_depth')
    if not isinstance(mcs_qos_depth, int) or mcs_qos_depth <= 0:
        raise StackConfigError('mcs.qos_depth must be a positive integer.')
    mcs_pose_topic(config, 'validation')

    refresh_rate_hz = required(config, 'waypoint_ui', 'refresh_rate_hz')
    if (
        not isinstance(refresh_rate_hz, (int, float))
        or isinstance(refresh_rate_hz, bool)
        or not math.isfinite(float(refresh_rate_hz))
        or refresh_rate_hz <= 0.0
    ):
        raise StackConfigError(
            'waypoint_ui.refresh_rate_hz must be a positive finite number.'
        )

    loiter_direction = str(required(
        config,
        'waypoint_controller',
        'final_loiter_direction',
    )).strip().lower()
    if loiter_direction not in ('left', 'right'):
        raise StackConfigError(
            'waypoint_controller.final_loiter_direction must be left or '
            'right.'
        )
    config['waypoint_controller'][
        'final_loiter_direction'
    ] = loiter_direction
    if 'robot_name' in config:
        raise StackConfigError(
            'Shared WaveRover stack configuration "%s" must not contain '
            'robot_name; put it in robot_identity.yaml.' % path
        )
    if require_identity:
        config.update(load_robot_identity(identity_path, path))
    return deepcopy(config)


def required(config, *keys):
    """Read a required nested configuration value."""
    value = config
    traversed = []
    for key in keys:
        traversed.append(key)
        if not isinstance(value, dict) or key not in value:
            raise StackConfigError(
                'Missing required WaveRover configuration key: %s'
                % '.'.join(traversed)
            )
        value = value[key]
    return value


def validate_robot_name(value):
    """Validate and normalize a deployment robot ID."""
    if value is None:
        raise StackConfigError('robot_name must not be empty.')
    robot_name = str(value).strip()
    if not ROBOT_NAME_PATTERN.fullmatch(robot_name):
        raise StackConfigError(
            'robot_name must contain only letters, digits, and underscores.'
        )
    return robot_name


def normalize_control_mode(value, supported=None):
    """Normalize a control mode and reject unsupported values."""
    control_mode = str(value).strip().lower()
    allowed = tuple(supported or SUPPORTED_CONTROL_MODES)
    if control_mode not in allowed:
        raise StackConfigError(
            'Invalid control_mode "%s"; expected one of: %s.'
            % (control_mode, ', '.join(allowed))
        )
    return control_mode


def normalize_pose_source(value):
    """Normalize a pose source and reject unknown values."""
    pose_source = str(value).strip().upper()
    if pose_source not in SUPPORTED_POSE_SOURCES:
        raise StackConfigError(
            'Invalid pose_source "%s"; expected SLAM or MCS.' % pose_source
        )
    return pose_source


def robot_namespace(config, robot_name=None):
    """Derive the relative ROS namespace for one robot."""
    selected_name = validate_robot_name(
        robot_name if robot_name is not None else required(
            config,
            'robot_name',
        )
    )
    return '%s%s' % (required(config, 'namespace_prefix'), selected_name)


def robot_frame(config, frame_key, robot_name=None):
    """Derive a collision-free frame ID from robot_name."""
    return '%s/%s' % (
        robot_namespace(config, robot_name),
        required(config, 'frames', frame_key),
    )


def robot_topic(config, topic_key, robot_name=None):
    """Derive an absolute robot topic from a configured relative name."""
    topic = str(required(config, 'topics', topic_key))
    if topic.startswith('/'):
        return topic
    return '/%s/%s' % (robot_namespace(config, robot_name), topic)


def mcs_pose_topic(config, robot_name=None):
    """Derive the external MCS PoseStamped topic for one robot."""
    selected_name = validate_robot_name(
        robot_name if robot_name is not None else required(
            config,
            'robot_name',
        )
    )
    namespace = robot_namespace(config, selected_name)
    pattern = required(config, 'mcs', 'pose_topic_pattern')
    if not isinstance(pattern, str) or not pattern:
        raise StackConfigError('mcs.pose_topic_pattern must be a string.')
    try:
        fields = {
            field_name
            for _, field_name, _, _ in Formatter().parse(pattern)
            if field_name is not None
        }
        if not fields.issubset({'robot_name', 'robot_namespace'}):
            raise StackConfigError(
                'mcs.pose_topic_pattern may use only {robot_name} and '
                '{robot_namespace}.'
            )
        topic = pattern.format(
            robot_name=selected_name,
            robot_namespace=namespace,
        )
    except (AttributeError, IndexError, KeyError, ValueError) as error:
        raise StackConfigError(
            'Invalid mcs.pose_topic_pattern: %s' % error
        ) from error
    if not topic.startswith('/') or '{' in topic or '}' in topic:
        raise StackConfigError(
            'mcs.pose_topic_pattern must produce an absolute topic and may '
            'use only {robot_name} and {robot_namespace}.'
        )
    return topic


def waypoint_global_frame(config, pose_source=None, robot_name=None):
    """Return the waypoint/pose global frame for the selected source."""
    selected_source = normalize_pose_source(
        pose_source if pose_source is not None else required(
            config,
            'pose_source',
        )
    )
    if selected_source == 'MCS':
        return str(required(config, 'mcs', 'frame'))
    return robot_frame(config, 'map', robot_name)


def launch_text(value):
    """Convert a YAML scalar into a ROS launch default string."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)
