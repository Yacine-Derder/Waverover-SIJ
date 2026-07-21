"""Strict experiment and target configuration loading."""

from dataclasses import dataclass
import math
from pathlib import Path
import re
import warnings

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
    refresh_period_sec: float
    active_waypoint_warning_sec: float
    repeated_destination_epsilon_m: float = 0.05
    completed_destination_reissue_distance_m: float = 0.30
    reached_distance_m: float = 0.0
    handoff_delay_sec: float = 0.0


@dataclass(frozen=True)
class TargetDynamicsConfig:
    mode: str
    switch_period_sec: float
    priority_weight: float
    background_weight: float
    seed: int
    initial_priority_target_id: object
    avoid_immediate_repeat: bool


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
    preferred_separation_m: float
    collision_policy: str
    collision_repair_max_iterations: int
    controller_result_timeout_sec: float
    geofence: GeofenceConfig

    @property
    def minimum_separation_m(self):
        """Backward-compatible name; separation is preferred, not fatal."""
        return self.preferred_separation_m


@dataclass(frozen=True)
class ManeuverSegment:
    action: str
    duration_sec: float


@dataclass(frozen=True)
class SyntheticMcsConfig:
    mode: str
    preset: str
    seed: object
    duration_sec: float
    formation_coupling: str
    initial_radius_m: float
    connectivity_policy: str
    segment_duration_min_sec: float
    segment_duration_max_sec: float
    process_speed_std_mps: float
    process_yaw_rate_std_rad_s: float
    measurement_position_std_m: float
    measurement_heading_std_rad: float
    maximum_transition_attempts: int
    script: tuple


@dataclass(frozen=True)
class RecordingConfig:
    root_directory: object
    profile: str
    storage_id: str
    pose_source: str
    start_synthetic: bool


@dataclass(frozen=True)
class AnalysisConfig:
    connectivity_alpha: float
    maximum_interpolation_gap_sec: float


@dataclass(frozen=True)
class ExperimentConfig:
    frame_id: str
    robot_ids: tuple
    pose: PoseConfig
    station: StationState
    targets: tuple
    priority_target_id: object
    target_dynamics: TargetDynamicsConfig
    vehicle: VehicleConfig
    controller: ControllerConfig
    waypoint_dispatch: DispatchConfig
    communication: CommunicationConfig
    safety: SafetyConfig
    synthetic_mcs: SyntheticMcsConfig
    recording: RecordingConfig
    analysis: AnalysisConfig
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


def _positive_integer(value, name):
    if isinstance(value, bool):
        raise ConfigError('%s must be an integer.' % name)
    try:
        number = int(value)
    except (TypeError, ValueError) as error:
        raise ConfigError('%s must be an integer.' % name) from error
    if isinstance(value, float) and not value.is_integer():
        raise ConfigError('%s must be an integer.' % name)
    if number <= 0:
        raise ConfigError('%s must be positive.' % name)
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
    legacy_main_target_id = str(data.get('main_target_id', '')).strip() or None
    if legacy_main_target_id is not None:
        warnings.warn(
            'main_target_id/static target weights are deprecated; '
            'target_dynamics controls runtime priority.',
            DeprecationWarning,
            stacklevel=2,
        )
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
            weight=_finite(
                target.get('weight', 1.0),
                'targets[%d].weight' % index,
                nonnegative=True,
            ),
            is_priority=target_id == legacy_main_target_id,
        ))
    if legacy_main_target_id is not None and sum(
        target.target_id == legacy_main_target_id for target in targets
    ) != 1:
        raise ConfigError('Exactly one target must match legacy main_target_id.')
    return tuple(targets), legacy_main_target_id


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
    deprecated_dispatch = {
        'reached_distance_m', 'handoff_delay_sec'
    }.intersection(dispatch_data)
    if deprecated_dispatch:
        warnings.warn(
            '%s no longer control PC handoff; onboard acknowledgement is '
            'authoritative.' % ', '.join(sorted(deprecated_dispatch)),
            DeprecationWarning,
            stacklevel=2,
        )
    communication_data = _mapping(data.get('communication'), 'communication')
    safety_data = _mapping(data.get('safety'), 'safety')
    synthetic_data = _mapping(data.get('synthetic_mcs', {}), 'synthetic_mcs')
    recording_data = _mapping(data.get('recording', {}), 'recording')
    analysis_data = _mapping(data.get('analysis', {}), 'analysis')
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
        controller_seed = int(controller_data.get('deterministic_seed', 42))
    except (TypeError, ValueError) as error:
        raise ConfigError('mpc_horizon and deterministic_seed must be integers.') from error
    if horizon <= 0:
        raise ConfigError('controller.mpc_horizon must be positive.')

    dry_run = safety_data.get('dry_run')
    if dry_run_override is not None:
        dry_run = bool(dry_run_override)
    if not isinstance(dry_run, bool):
        raise ConfigError('safety.dry_run must be boolean.')
    collision_policy = str(
        safety_data.get('collision_policy', 'best_effort')
    ).strip().lower()
    if collision_policy != 'best_effort':
        raise ConfigError('safety.collision_policy must be best_effort.')
    preferred_separation = safety_data.get(
        'preferred_separation_m', safety_data.get('minimum_separation_m', 0.30)
    )

    synthetic_mode = str(synthetic_data.get('mode', 'static')).strip().lower()
    if synthetic_mode not in (
        'static', 'scripted', 'preset', 'random_walk', 'noisy_path'
    ):
        raise ConfigError(
            'synthetic_mcs.mode must be static, scripted, preset, '
            'random_walk, or noisy_path.'
        )
    preset = str(synthetic_data.get('preset', 'figure_eight')).strip().lower()
    if preset not in ('circle', 'racetrack', 'figure_eight'):
        raise ConfigError(
            'synthetic_mcs.preset must be circle, racetrack, or figure_eight.'
        )
    seed = synthetic_data.get('seed')
    if seed is not None:
        if isinstance(seed, bool):
            raise ConfigError('synthetic_mcs.seed must be an integer or null.')
        try:
            seed = int(seed)
        except (TypeError, ValueError) as error:
            raise ConfigError(
                'synthetic_mcs.seed must be an integer or null.'
            ) from error
    formation_coupling = str(
        synthetic_data.get('formation_coupling', 'rigid')
    ).strip().lower()
    if formation_coupling not in ('rigid', 'independent'):
        raise ConfigError(
            'synthetic_mcs.formation_coupling must be rigid or independent.'
        )
    connectivity_policy = str(
        synthetic_data.get('connectivity_policy', 'enforce')
    ).strip().lower()
    if connectivity_policy not in ('enforce', 'observe'):
        raise ConfigError(
            'synthetic_mcs.connectivity_policy must be enforce or observe.'
        )
    actions = {'straight', 'bank_left', 'bank_right'}
    script = []
    raw_script = synthetic_data.get('script', [])
    if not isinstance(raw_script, list):
        raise ConfigError('synthetic_mcs.script must be a list.')
    for index, raw_segment in enumerate(raw_script):
        segment = _mapping(raw_segment, 'synthetic_mcs.script[%d]' % index)
        action = str(segment.get('action', '')).strip().lower()
        if action not in actions:
            raise ConfigError(
                'synthetic_mcs.script[%d].action must be straight, '
                'bank_left, or bank_right.' % index
            )
        script.append(ManeuverSegment(
            action=action,
            duration_sec=_finite(
                segment.get('duration_sec'),
                'synthetic_mcs.script[%d].duration_sec' % index,
                positive=True,
            ),
        ))
    if synthetic_mode == 'scripted' and not script:
        raise ConfigError('synthetic_mcs.scripted mode requires a script.')
    segment_min = _finite(
        synthetic_data.get('segment_duration_min_sec', 1.0),
        'synthetic_mcs.segment_duration_min_sec',
        positive=True,
    )
    segment_max = _finite(
        synthetic_data.get('segment_duration_max_sec', 5.0),
        'synthetic_mcs.segment_duration_max_sec',
        positive=True,
    )
    if segment_min > segment_max:
        raise ConfigError(
            'synthetic_mcs segment duration minimum must not exceed maximum.'
        )

    recording_profile = str(
        recording_data.get('profile', 'core')
    ).strip().lower()
    if recording_profile not in ('core', 'full'):
        raise ConfigError('recording.profile must be core or full.')
    storage_id = str(recording_data.get('storage_id', 'sqlite3')).strip()
    if not storage_id:
        raise ConfigError('recording.storage_id must be nonempty.')
    pose_source = str(
        recording_data.get('pose_source', 'synthetic')
    ).strip().lower()
    if pose_source not in ('synthetic', 'mcs'):
        raise ConfigError('recording.pose_source must be synthetic or mcs.')
    start_synthetic = recording_data.get('start_synthetic', True)
    if not isinstance(start_synthetic, bool):
        raise ConfigError('recording.start_synthetic must be boolean.')
    root_directory = recording_data.get('root_directory')
    if root_directory is not None:
        root_directory = str(root_directory).strip()
        if not root_directory:
            raise ConfigError(
                'recording.root_directory must be nonempty or null.'
            )

    targets_value = str(data.get('targets_file', '')).strip()
    if not targets_value:
        raise ConfigError('targets_file must be nonempty.')
    targets_path = Path(targets_value)
    if not targets_path.is_absolute():
        targets_path = source_path.parent / targets_path
    targets_path = targets_path.resolve()
    targets, legacy_priority_target_id = load_targets(targets_path, geofence)
    if 'target_dynamics' in data:
        target_source = _read_yaml(targets_path, 'targets file')
        if 'main_target_id' in target_source or any(
            isinstance(entry, dict) and 'weight' in entry
            for entry in target_source.get('targets', [])
        ):
            raise ConfigError(
                'New target_dynamics experiments require neutral target '
                'definitions without main_target_id or static weights.'
            )
    dynamics_data = _mapping(data.get('target_dynamics', {}), 'target_dynamics')
    dynamics_mode = str(dynamics_data.get('mode', 'random_priority')).strip().lower()
    if dynamics_mode != 'random_priority':
        raise ConfigError('target_dynamics.mode must be random_priority.')
    priority_weight = _finite(
        dynamics_data.get('priority_weight', 10.0),
        'target_dynamics.priority_weight', positive=True,
    )
    background_weight = _finite(
        dynamics_data.get('background_weight', 1.0),
        'target_dynamics.background_weight', positive=True,
    )
    if priority_weight < background_weight:
        raise ConfigError('priority_weight must be >= background_weight.')
    initial_priority = dynamics_data.get(
        'initial_priority_target_id', legacy_priority_target_id
    )
    if initial_priority is not None:
        initial_priority = str(initial_priority).strip()
        if initial_priority not in {target.target_id for target in targets}:
            raise ConfigError('initial_priority_target_id must identify a target.')
    avoid_repeat = dynamics_data.get('avoid_immediate_repeat', True)
    if not isinstance(avoid_repeat, bool):
        raise ConfigError('target_dynamics.avoid_immediate_repeat must be boolean.')
    try:
        target_seed = int(dynamics_data.get('seed', 2026))
    except (TypeError, ValueError) as error:
        raise ConfigError('target_dynamics.seed must be an integer.') from error

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

    vehicle = VehicleConfig(
        straight_speed_mps=_finite(
            vehicle_data.get('straight_speed_mps'),
            'vehicle.straight_speed_mps',
            positive=True,
        ),
        turn_radius_m=_finite(
            vehicle_data.get('turn_radius_m'),
            'vehicle.turn_radius_m',
            positive=True,
        ),
        bank_yaw_rate_rad_s=_finite(
            vehicle_data.get('bank_yaw_rate_rad_s'),
            'vehicle.bank_yaw_rate_rad_s',
            positive=True,
        ),
        turning_path_speed_mps=_finite(
            vehicle_data.get('turning_path_speed_mps'),
            'vehicle.turning_path_speed_mps',
            positive=True,
        ),
    )
    implied_radius = (
        vehicle.turning_path_speed_mps / vehicle.bank_yaw_rate_rad_s
    )
    radius_tolerance = max(1e-4, 0.05 * vehicle.turn_radius_m)
    if abs(implied_radius - vehicle.turn_radius_m) > radius_tolerance:
        raise ConfigError(
            'vehicle turning_path_speed_mps / bank_yaw_rate_rad_s implies '
            'turn radius %.6f m, inconsistent with turn_radius_m %.6f m.'
            % (implied_radius, vehicle.turn_radius_m)
        )

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
        priority_target_id=initial_priority,
        target_dynamics=TargetDynamicsConfig(
            mode=dynamics_mode,
            switch_period_sec=_finite(
                dynamics_data.get('switch_period_sec', 20.0),
                'target_dynamics.switch_period_sec', positive=True,
            ),
            priority_weight=priority_weight,
            background_weight=background_weight,
            seed=target_seed,
            initial_priority_target_id=initial_priority,
            avoid_immediate_repeat=avoid_repeat,
        ),
        vehicle=vehicle,
        controller=ControllerConfig(
            algorithm=algorithm,
            control_period_sec=_finite(
                controller_data.get('control_period_sec'),
                'controller.control_period_sec', positive=True,
            ),
            mpc_horizon=horizon,
            mpc_max_step_m=_finite(
                controller_data.get('mpc_max_step_m'),
                'controller.mpc_max_step_m', positive=True,
            ),
            minimum_mpc_lookahead_m=_finite(
                controller_data.get('minimum_mpc_lookahead_m'),
                'controller.minimum_mpc_lookahead_m', positive=True,
            ),
            deterministic_seed=controller_seed,
            distributed_update_semantics=semantics,
        ),
        waypoint_dispatch=DispatchConfig(
            refresh_period_sec=_finite(
                dispatch_data.get('refresh_period_sec', 1.0),
                'waypoint_dispatch.refresh_period_sec',
                positive=True,
            ),
            active_waypoint_warning_sec=_finite(
                dispatch_data.get(
                    'active_waypoint_warning_sec',
                    dispatch_data.get('maximum_active_time_sec', 10.0),
                ),
                'waypoint_dispatch.active_waypoint_warning_sec',
                positive=True,
            ),
            repeated_destination_epsilon_m=_finite(
                dispatch_data.get('repeated_destination_epsilon_m', 0.05),
                'waypoint_dispatch.repeated_destination_epsilon_m',
                positive=True,
            ),
            completed_destination_reissue_distance_m=_finite(
                dispatch_data.get(
                    'completed_destination_reissue_distance_m', 0.30
                ),
                'waypoint_dispatch.completed_destination_reissue_distance_m',
                positive=True,
            ),
            reached_distance_m=float(dispatch_data.get('reached_distance_m', 0.0)),
            handoff_delay_sec=float(dispatch_data.get('handoff_delay_sec', 0.0)),
        ),
        communication=CommunicationConfig(ideal_range, maximum_range),
        safety=SafetyConfig(
            dry_run=dry_run,
            preferred_separation_m=_finite(
                preferred_separation,
                'safety.preferred_separation_m',
                positive=True,
            ),
            collision_policy=collision_policy,
            collision_repair_max_iterations=_positive_integer(
                safety_data.get('collision_repair_max_iterations', 50),
                'safety.collision_repair_max_iterations',
            ),
            controller_result_timeout_sec=_finite(
                safety_data.get('controller_result_timeout_sec', 2.5),
                'safety.controller_result_timeout_sec', positive=True,
            ),
            geofence=geofence,
        ),
        synthetic_mcs=SyntheticMcsConfig(
            mode=synthetic_mode,
            preset=preset,
            seed=seed,
            duration_sec=_finite(
                synthetic_data.get('duration_sec', 120.0),
                'synthetic_mcs.duration_sec',
                positive=True,
            ),
            formation_coupling=formation_coupling,
            initial_radius_m=_finite(
                synthetic_data.get('initial_radius_m', 0.5),
                'synthetic_mcs.initial_radius_m',
                nonnegative=True,
            ),
            connectivity_policy=connectivity_policy,
            segment_duration_min_sec=segment_min,
            segment_duration_max_sec=segment_max,
            process_speed_std_mps=_finite(
                synthetic_data.get('process_speed_std_mps', 0.0),
                'synthetic_mcs.process_speed_std_mps',
                nonnegative=True,
            ),
            process_yaw_rate_std_rad_s=_finite(
                synthetic_data.get('process_yaw_rate_std_rad_s', 0.0),
                'synthetic_mcs.process_yaw_rate_std_rad_s',
                nonnegative=True,
            ),
            measurement_position_std_m=_finite(
                synthetic_data.get('measurement_position_std_m', 0.0),
                'synthetic_mcs.measurement_position_std_m',
                nonnegative=True,
            ),
            measurement_heading_std_rad=_finite(
                synthetic_data.get('measurement_heading_std_rad', 0.0),
                'synthetic_mcs.measurement_heading_std_rad',
                nonnegative=True,
            ),
            maximum_transition_attempts=_positive_integer(
                synthetic_data.get('maximum_transition_attempts', 50),
                'synthetic_mcs.maximum_transition_attempts',
            ),
            script=tuple(script),
        ),
        recording=RecordingConfig(
            root_directory=root_directory,
            profile=recording_profile,
            storage_id=storage_id,
            pose_source=pose_source,
            start_synthetic=start_synthetic,
        ),
        analysis=AnalysisConfig(
            connectivity_alpha=_finite(
                analysis_data.get('connectivity_alpha', 5.0),
                'analysis.connectivity_alpha',
                positive=True,
            ),
            maximum_interpolation_gap_sec=_finite(
                analysis_data.get('maximum_interpolation_gap_sec', 0.5),
                'analysis.maximum_interpolation_gap_sec',
                positive=True,
            ),
        ),
        source_path=source_path,
        targets_path=targets_path,
    )
