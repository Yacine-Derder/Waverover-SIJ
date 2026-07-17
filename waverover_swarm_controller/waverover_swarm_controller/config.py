"""Strict experiment and target configuration loading."""

from dataclasses import dataclass
import math
from pathlib import Path
import re

import yaml

from .models import StationState, TargetState


ROBOT_ID_PATTERN = re.compile(r'[A-Za-z0-9_]+')
SUPPORTED_ALGORITHMS = (
    'heuristic',
    'heuristic_decentralized',
    'convex',
    'mpc_centralized',
    'mpc_distributed',
)


class ConfigError(ValueError):
    """Raised when an experiment cannot be operated safely."""


@dataclass(frozen=True)
class PoseConfig:
    timeout_sec: float
    maximum_snapshot_skew_sec: float


@dataclass(frozen=True)
class VehicleConfig:
    straight_speed_mps: float
    turn_radius_m: float
    bank_yaw_rate_rad_s: float
    turning_path_speed_mps: float


@dataclass(frozen=True)
class ControllerConfig:
    algorithm: str
    control_period_sec: float
    mpc_horizon: int
    mpc_max_step_m: float
    minimum_mpc_lookahead_m: float
    deterministic_seed: int
    distributed_update_semantics: str


@dataclass(frozen=True)
class DispatchConfig:
    reached_distance_m: float
    handoff_delay_sec: float
    maximum_active_time_sec: float


@dataclass(frozen=True)
class CommunicationConfig:
    ideal_range_m: float
    maximum_range_m: float


@dataclass(frozen=True)
class GeofenceConfig:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, point):
        x, y = point
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass(frozen=True)
class SafetyConfig:
    dry_run: bool
    minimum_separation_m: float
    controller_result_timeout_sec: float
    geofence: GeofenceConfig


@dataclass(frozen=True)
class ExperimentConfig:
    frame_id: str
    robot_ids: tuple
    pose: PoseConfig
    station: StationState
    targets: tuple
    main_target_id: str
    vehicle: VehicleConfig
    controller: ControllerConfig
    waypoint_dispatch: DispatchConfig
    communication: CommunicationConfig
    safety: SafetyConfig
    source_path: Path
    targets_path: Path


def _mapping(value, name):
    if not isinstance(value, dict):
        raise ConfigError('%s must be a mapping.' % name)
    return value


def _finite(value, name, *, positive=False, nonnegative=False):
    if isinstance(value, bool):
        raise ConfigError('%s must be numeric.' % name)
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ConfigError('%s must be numeric.' % name) from error
    if not math.isfinite(number):
        raise ConfigError('%s must be finite.' % name)
    if positive and number <= 0.0:
        raise ConfigError('%s must be positive.' % name)
    if nonnegative and number < 0.0:
        raise ConfigError('%s must be nonnegative.' % name)
    return number


def _point(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ConfigError('%s must contain exactly [x, y].' % name)
    return (
        _finite(value[0], name + '[0]'),
        _finite(value[1], name + '[1]'),
    )


def _read_yaml(path, description):
    try:
        with Path(path).open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except OSError as error:
        raise ConfigError('Could not read %s %s: %s' % (description, path, error)) from error
    except yaml.YAMLError as error:
        raise ConfigError('Could not parse %s %s: %s' % (description, path, error)) from error
    return _mapping(data, description)


def load_targets(path, geofence):
    data = _read_yaml(path, 'targets file')
    if data.get('frame_id') != 'robotics_lab':
        raise ConfigError('targets frame_id must be robotics_lab.')
    main_target_id = str(data.get('main_target_id', '')).strip()
    if not main_target_id:
        raise ConfigError('main_target_id must be nonempty.')
    entries = data.get('targets')
    if not isinstance(entries, list) or not entries:
        raise ConfigError('targets must be a nonempty list.')

    targets = []
    seen = set()
    for index, raw_target in enumerate(entries):
        target = _mapping(raw_target, 'targets[%d]' % index)
        target_id = str(target.get('id', '')).strip()
        if not target_id or target_id in seen:
            raise ConfigError('Target IDs must be unique and nonempty.')
        seen.add(target_id)
        position = _point(target.get('position'), 'targets[%d].position' % index)
        if not geofence.contains(position):
            raise ConfigError('Target %s falls outside the configured geofence.' % target_id)
        targets.append(TargetState(
            target_id=target_id,
            x=position[0],
            y=position[1],
            weight=_finite(target.get('weight'), 'targets[%d].weight' % index, nonnegative=True),
            is_main=target_id == main_target_id,
        ))
    if sum(target.target_id == main_target_id for target in targets) != 1:
        raise ConfigError('Exactly one target must match main_target_id.')
    return tuple(targets), main_target_id


def load_experiment(path, algorithm_override=None, dry_run_override=None):
    source_path = Path(path).expanduser().resolve()
    data = _read_yaml(source_path, 'experiment configuration')
    if data.get('frame_id') != 'robotics_lab':
        raise ConfigError('frame_id must be robotics_lab.')

    raw_robot_ids = data.get('robot_ids')
    if not isinstance(raw_robot_ids, list) or not raw_robot_ids:
        raise ConfigError('robot_ids must be a nonempty list.')
    robot_ids = tuple(str(value).strip() for value in raw_robot_ids)
    if any(not ROBOT_ID_PATTERN.fullmatch(value) for value in robot_ids):
        raise ConfigError('robot_ids may contain only letters, digits, and underscores.')
    if len(set(robot_ids)) != len(robot_ids):
        raise ConfigError('robot_ids must be unique.')

    pose_data = _mapping(data.get('pose'), 'pose')
    station_data = _mapping(data.get('station'), 'station')
    station_id = str(station_data.get('id', '')).strip()
    if not station_id:
        raise ConfigError('station.id must be nonempty.')
    station_position = _point(station_data.get('position'), 'station.position')

    vehicle_data = _mapping(data.get('vehicle'), 'vehicle')
    controller_data = _mapping(data.get('controller'), 'controller')
    dispatch_data = _mapping(data.get('waypoint_dispatch'), 'waypoint_dispatch')
    communication_data = _mapping(data.get('communication'), 'communication')
    safety_data = _mapping(data.get('safety'), 'safety')
    geofence_data = _mapping(safety_data.get('geofence'), 'safety.geofence')
    geofence = GeofenceConfig(
        x_min=_finite(geofence_data.get('x_min'), 'safety.geofence.x_min'),
        x_max=_finite(geofence_data.get('x_max'), 'safety.geofence.x_max'),
        y_min=_finite(geofence_data.get('y_min'), 'safety.geofence.y_min'),
        y_max=_finite(geofence_data.get('y_max'), 'safety.geofence.y_max'),
    )
    if geofence.x_min >= geofence.x_max or geofence.y_min >= geofence.y_max:
        raise ConfigError('Geofence minima must be less than maxima.')
    if not geofence.contains(station_position):
        raise ConfigError('Station falls outside the configured geofence.')

    algorithm = str(
        algorithm_override or controller_data.get('algorithm', '')
    ).strip().lower()
    if algorithm not in SUPPORTED_ALGORITHMS:
        raise ConfigError(
            'controller.algorithm must be one of: %s.'
            % ', '.join(SUPPORTED_ALGORITHMS)
        )
    semantics = str(controller_data.get(
        'distributed_update_semantics', 'jacobi'
    )).strip().lower()
    if semantics not in ('jacobi', 'gauss_seidel'):
        raise ConfigError(
            'distributed_update_semantics must be jacobi or gauss_seidel.'
        )
    try:
        horizon = int(controller_data.get('mpc_horizon'))
        seed = int(controller_data.get('deterministic_seed', 42))
    except (TypeError, ValueError) as error:
        raise ConfigError('mpc_horizon and deterministic_seed must be integers.') from error
    if horizon <= 0:
        raise ConfigError('controller.mpc_horizon must be positive.')

    dry_run = safety_data.get('dry_run')
    if dry_run_override is not None:
        dry_run = bool(dry_run_override)
    if not isinstance(dry_run, bool):
        raise ConfigError('safety.dry_run must be boolean.')

    targets_value = str(data.get('targets_file', '')).strip()
    if not targets_value:
        raise ConfigError('targets_file must be nonempty.')
    targets_path = Path(targets_value)
    if not targets_path.is_absolute():
        targets_path = source_path.parent / targets_path
    targets_path = targets_path.resolve()
    targets, main_target_id = load_targets(targets_path, geofence)

    ideal_range = _finite(
        communication_data.get('ideal_range_m'),
        'communication.ideal_range_m',
        positive=True,
    )
    maximum_range = _finite(
        communication_data.get('maximum_range_m'),
        'communication.maximum_range_m',
        positive=True,
    )
    if ideal_range > maximum_range:
        raise ConfigError('ideal_range_m must not exceed maximum_range_m.')

    return ExperimentConfig(
        frame_id='robotics_lab',
        robot_ids=robot_ids,
        pose=PoseConfig(
            timeout_sec=_finite(pose_data.get('timeout_sec'), 'pose.timeout_sec', positive=True),
            maximum_snapshot_skew_sec=_finite(
                pose_data.get('maximum_snapshot_skew_sec'),
                'pose.maximum_snapshot_skew_sec',
                nonnegative=True,
            ),
        ),
        station=StationState(station_id, *station_position),
        targets=targets,
        main_target_id=main_target_id,
        vehicle=VehicleConfig(
            straight_speed_mps=_finite(vehicle_data.get('straight_speed_mps'), 'vehicle.straight_speed_mps', positive=True),
            turn_radius_m=_finite(vehicle_data.get('turn_radius_m'), 'vehicle.turn_radius_m', positive=True),
            bank_yaw_rate_rad_s=_finite(vehicle_data.get('bank_yaw_rate_rad_s'), 'vehicle.bank_yaw_rate_rad_s', positive=True),
            turning_path_speed_mps=_finite(vehicle_data.get('turning_path_speed_mps'), 'vehicle.turning_path_speed_mps', positive=True),
        ),
        controller=ControllerConfig(
            algorithm=algorithm,
            control_period_sec=_finite(controller_data.get('control_period_sec'), 'controller.control_period_sec', positive=True),
            mpc_horizon=horizon,
            mpc_max_step_m=_finite(controller_data.get('mpc_max_step_m'), 'controller.mpc_max_step_m', positive=True),
            minimum_mpc_lookahead_m=_finite(controller_data.get('minimum_mpc_lookahead_m'), 'controller.minimum_mpc_lookahead_m', positive=True),
            deterministic_seed=seed,
            distributed_update_semantics=semantics,
        ),
        waypoint_dispatch=DispatchConfig(
            reached_distance_m=_finite(dispatch_data.get('reached_distance_m'), 'waypoint_dispatch.reached_distance_m', positive=True),
            handoff_delay_sec=_finite(dispatch_data.get('handoff_delay_sec'), 'waypoint_dispatch.handoff_delay_sec', nonnegative=True),
            maximum_active_time_sec=_finite(dispatch_data.get('maximum_active_time_sec'), 'waypoint_dispatch.maximum_active_time_sec', positive=True),
        ),
        communication=CommunicationConfig(ideal_range, maximum_range),
        safety=SafetyConfig(
            dry_run=dry_run,
            minimum_separation_m=_finite(safety_data.get('minimum_separation_m'), 'safety.minimum_separation_m', positive=True),
            controller_result_timeout_sec=_finite(safety_data.get('controller_result_timeout_sec', 2.5), 'safety.controller_result_timeout_sec', positive=True),
            geofence=geofence,
        ),
        source_path=source_path,
        targets_path=targets_path,
    )
