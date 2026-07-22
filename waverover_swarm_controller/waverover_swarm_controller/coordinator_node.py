"""ROS boundary for the operator-PC swarm coordinator."""

from dataclasses import asdict, replace
import json
import math
import signal
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray
from waverover.stack_config import robot_namespace

from .config import ConfigError, load_experiment
from .controllers import controller_from_config
from .controllers.base import (
    complete_finite_mapping,
    controller_outcome_modes,
    controller_schedule,
    ControllerSchedule,
    ControllerUnavailableError,
    deterministic_connectivity_setpoints,
    optimization_hard_link_limit,
)
from .metrics import algebraic_connectivity, minimum_pairwise_distance
from .models import ControllerExecutionOutcome, ControllerResult
from .pose_aggregation import PoseAggregator, SnapshotUnavailableError
from .safety import SafetyViolation, validate_controller_result
from .target_dynamics import TargetManager
from .telemetry import build_controller_telemetry, canonical_json
from .waypoint_dispatcher import WaypointDispatcher


def predicted_path_topic(stack_config, robot_id):
    """Return a relative, ROS-valid predicted-path topic for one rover."""
    return 'predicted_path/' + robot_namespace(stack_config, robot_id)


class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('waverover_swarm_controller')
        config_file = str(self.declare_parameter('config_file', '').value).strip()
        algorithm = str(self.declare_parameter('algorithm', '').value).strip()
        dry_run = bool(self.declare_parameter('dry_run', True).value)
        if not config_file:
            raise ConfigError('config_file is required.')
        self.config = load_experiment(
            config_file,
            algorithm_override=algorithm or None,
            dry_run_override=dry_run,
        )
        self.controller = controller_from_config(self.config)
        self.target_manager = TargetManager(
            self.config.targets, self.config.target_dynamics,
            start_time=time.monotonic(),
        )
        self.dispatcher = WaypointDispatcher(
            self.config.robot_ids, self.config.waypoint_dispatch,
            self.config.safety,
        )
        self.aggregator = PoseAggregator(
            self.config.robot_ids,
            self.config.frame_id,
            self.config.pose.timeout_sec,
            self.config.pose.maximum_snapshot_skew_sec,
            logger=self.get_logger(),
        )
        self.latest_result = None
        self.latest_rejected_result = None
        self.latest_snapshot = None
        self.latest_stop_reason = 'startup: awaiting fresh synchronized poses'
        self.latest_handoff = 'none'
        self.latest_collision_events = []
        self._cleanup_complete = False
        self._end_sent_for_stop = False
        self._ack_warning_times = {}
        self.onboard_health = {}
        self.consecutive_recovery_cycles = 0
        self.fallback_counters = {}
        self.latest_execution_outcome = None
        self._last_controller_mode = None
        self._last_objective_signature = None
        self._objective_revision = 0
        self._last_compute_reason = 'not_computed'
        self._controller_compute_count = 0

        # The PC intentionally loads fleet naming without rover-local identity.
        from waverover.stack_config import (
            load_stack_config,
            mcs_pose_topic,
            robot_topic,
        )
        stack_config = load_stack_config(require_identity=False)
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.waypoint_publishers = {}
        self.end_trial_publishers = {}
        self.pose_subscriptions = []
        self.acknowledgement_subscriptions = []
        self.failure_subscriptions = []
        self.health_subscriptions = []
        for robot_id in self.config.robot_ids:
            self.waypoint_publishers[robot_id] = self.create_publisher(
                PointStamped,
                robot_topic(stack_config, 'waypoints', robot_id),
                reliable,
            )
            self.end_trial_publishers[robot_id] = self.create_publisher(
                Empty,
                robot_topic(stack_config, 'end_trial', robot_id),
                reliable,
            )
            topic = mcs_pose_topic(stack_config, robot_id)
            self.pose_subscriptions.append(self.create_subscription(
                PoseStamped,
                topic,
                lambda message, selected=robot_id: self.aggregator.receive(
                    selected, message
                ),
                best_effort,
            ))
            self.acknowledgement_subscriptions.append(self.create_subscription(
                PointStamped,
                robot_topic(stack_config, 'waypoint_reached', robot_id),
                lambda message, selected=robot_id: self._acknowledgement(
                    selected, message
                ),
                reliable,
            ))
            self.failure_subscriptions.append(self.create_subscription(
                PointStamped,
                robot_topic(stack_config, 'waypoint_failed', robot_id),
                lambda message, selected=robot_id: self._waypoint_failed(
                    selected, message
                ),
                reliable,
            ))
            self.health_subscriptions.append(self.create_subscription(
                String,
                robot_topic(stack_config, 'health', robot_id),
                lambda message, selected=robot_id: self._onboard_health(
                    selected, message
                ),
                reliable,
            ))

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, 'diagnostics', reliable
        )
        self.telemetry_publisher = self.create_publisher(
            String, 'controller_telemetry', reliable
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_state_publisher = self.create_publisher(
            String, 'target_state', target_qos
        )
        initial_targets, initial_state = self.target_manager.snapshot(
            time.monotonic()
        )
        self._publish_target_values(initial_targets, initial_state)
        self._published_target_epoch = initial_state['target_epoch']
        self.markers_publisher = self.create_publisher(
            MarkerArray, 'markers', reliable
        )
        self.path_publishers = {
            robot_id: self.create_publisher(
                Path,
                predicted_path_topic(stack_config, robot_id),
                reliable,
            )
            for robot_id in self.config.robot_ids
        }
        self.add_on_set_parameters_callback(self._parameters_callback)
        self.timer = self.create_timer(
            self.config.controller.control_period_sec,
            self._control_cycle,
        )
        self.get_logger().warn(
            'PC-only coordinator algorithm=%s dry_run=%s. %s'
            % (
                self.config.controller.algorithm,
                self.config.safety.dry_run,
                'Commands are suppressed.' if self.config.safety.dry_run else
                'Validated waypoints will dispatch automatically.',
            )
        )

    def _parameters_callback(self, parameters):
        for parameter in parameters:
            if parameter.name in ('config_file', 'dry_run', 'algorithm'):
                return SetParametersResult(
                    successful=False,
                    reason='%s changes require a coordinator relaunch.'
                    % parameter.name,
                )
        return SetParametersResult(successful=True)

    def _snapshot(self):
        now = time.monotonic()
        targets, state = self.target_manager.snapshot(now)
        self._target_state = state
        if state['target_epoch'] != self._published_target_epoch:
            self._publish_target_values(targets, state)
            self._published_target_epoch = state['target_epoch']
        return self.aggregator.snapshot(
            targets,
            self.config.station,
            now=now,
            priority_target_id=state['priority_target_id'],
            target_epoch=state['target_epoch'],
            target_epoch_started_at=state['target_epoch_started_at'],
            target_switch_reason=state['switch_reason'],
            target_selection_seed=state['target_selection_seed'],
            next_target_switch_at=state['next_switch_at'],
        )

    @staticmethod
    def _failure(error):
        return {
            'type': type(error).__name__,
            'message': str(error),
        }

    @staticmethod
    def _mission_objective_signature(snapshot):
        """Return deterministic semantic inputs used by static controllers."""
        return (
            (
                snapshot.station.station_id,
                float(snapshot.station.x),
                float(snapshot.station.y),
            ),
            snapshot.priority_target_id,
            tuple(
                (
                    target_id,
                    float(target.x),
                    float(target.y),
                    float(target.weight),
                    bool(target.is_priority),
                )
                for target_id, target in sorted(snapshot.targets.items())
            ),
        )

    def _computation_reason(self, snapshot):
        signature = SwarmCoordinator._mission_objective_signature(snapshot)
        schedule = controller_schedule(self.config.controller.algorithm)
        if getattr(self, '_last_objective_signature', None) is None:
            self._last_objective_signature = signature
            self._objective_revision = 1
            return 'startup'
        if signature != self._last_objective_signature:
            self._last_objective_signature = signature
            self._objective_revision = getattr(
                self, '_objective_revision', 0
            ) + 1
            return 'mission_objective_changed'
        if schedule is ControllerSchedule.RECEDING_HORIZON:
            return 'periodic_mpc'
        return None

    def _finish_outcome(self, result, dispatch_allowed, complete, validated,
                        mode, failures):
        # Keep outcome reporting best-effort: it must never replace the
        # controller failure which led us into recovery or safe hold.
        failures = dict(failures)
        counters = dict(getattr(self, 'fallback_counters', {}))
        if mode.startswith('normal_') or mode in (
            'heuristic', 'heuristic_decentralized'
        ):
            consecutive = 0
        elif mode in ('pose_unavailable', 'controller_error'):
            consecutive = getattr(self, 'consecutive_recovery_cycles', 0)
        else:
            consecutive = getattr(self, 'consecutive_recovery_cycles', 0) + 1
        self.consecutive_recovery_cycles = consecutive
        self.fallback_counters = counters
        previous = getattr(self, '_last_controller_mode', None)
        if previous != mode:
            try:
                logger = (
                    self.get_logger() if hasattr(self, 'get_logger') else None
                )
            except Exception as error:
                logger = None
                failures['outcome_reporting_error'] = (
                    SwarmCoordinator._failure(error)
                )
            if logger is not None:
                message = 'Controller mode transition: %s -> %s' % (
                    previous or 'startup', mode
                )
                try:
                    # ROS 2 Jazzy binds severity to a Python logging call site.
                    # These must remain distinct, static invocation lines.
                    if mode.startswith('normal_'):
                        logger.info(message)
                    else:
                        logger.warning(message)
                except Exception as error:
                    failures['outcome_reporting_error'] = (
                        SwarmCoordinator._failure(error)
                    )
            self._last_controller_mode = mode
        outcome = ControllerExecutionOutcome(
            result=result,
            dispatch_allowed=dispatch_allowed,
            complete_command_set_generated=complete,
            final_command_set_passed_validation=validated,
            controller_mode=mode,
            failure_metadata=failures,
            consecutive_recovery_cycles=consecutive,
            fallback_counters=counters,
        )
        self.latest_execution_outcome = outcome
        return outcome

    def _note_fallback_attempt(self, mode):
        counters = dict(getattr(self, 'fallback_counters', {}))
        counters[mode] = counters.get(mode, 0) + 1
        self.fallback_counters = counters

    def _process_candidate(self, snapshot, result):
        if not complete_finite_mapping(snapshot, result):
            raise SafetyViolation(
                'Controller candidate is incomplete or non-finite.'
            )
        diagnostics = dict(result.controller_diagnostics)
        for field in (
            'maximum_connectivity_slack_m', 'total_connectivity_slack_m'
        ):
            if field in diagnostics and (
                not math.isfinite(float(diagnostics[field]))
                or float(diagnostics[field]) < 0.0
            ):
                raise SafetyViolation(
                    '%s must be finite and nonnegative.' % field
                )
        collision_validation = validate_controller_result(
            self.config, snapshot, result, time.monotonic()
        )
        self.latest_collision_events = (
            [] if collision_validation is True else collision_validation
        )
        return result

    def _compute_valid_result(self, snapshot):
        modes = controller_outcome_modes(self.config.controller.algorithm)
        if modes is None:
            result = SwarmCoordinator._process_candidate(
                self, snapshot, self.controller.compute(snapshot)
            )
            return SwarmCoordinator._finish_outcome(
                self,
                result, True, True, True,
                self.config.controller.algorithm, {},
            )
        normal_mode, recovery_mode = modes
        attempts = [(normal_mode, self.controller.compute)]
        recovery = getattr(self.controller, 'compute_recovery', None)
        if recovery is not None:
            attempts.append((recovery_mode, recovery))
        failures = {
            'normal_solver_status': None,
            'recovery_solver_status': None,
            'normal_failure_reason': '',
            'recovery_failure_reason': '',
            'controller_exception': None,
            'maximum_connectivity_slack_m': 0.0,
            'total_connectivity_slack_m': 0.0,
            'selected_edges_replaced': False,
        }
        selected_edges = ()
        for mode, operation in attempts:
            if mode == recovery_mode:
                SwarmCoordinator._note_fallback_attempt(self, mode)
            try:
                result = operation(snapshot)
                selected_edges = result.selected_edges or selected_edges
                status_key = (
                    'normal_solver_status'
                    if mode == normal_mode else 'recovery_solver_status'
                )
                failures[status_key] = result.solver_status
                diagnostics = dict(result.controller_diagnostics)
                failures['maximum_connectivity_slack_m'] = diagnostics.get(
                    'maximum_connectivity_slack_m', 0.0
                )
                failures['total_connectivity_slack_m'] = diagnostics.get(
                    'total_connectivity_slack_m', 0.0
                )
                result = SwarmCoordinator._process_candidate(
                    self, snapshot, result
                )
                self.latest_rejected_result = None
                return SwarmCoordinator._finish_outcome(
                    self,
                    result, True, True, True, mode, failures
                )
            except Exception as error:
                failure = SwarmCoordinator._failure(error)
                reason_key = (
                    'normal_failure_reason'
                    if mode == normal_mode else 'recovery_failure_reason'
                )
                status_key = (
                    'normal_solver_status'
                    if mode == normal_mode else 'recovery_solver_status'
                )
                failures[reason_key] = failure['message']
                failures['controller_exception'] = failure
                failures[
                    'normal_controller_exception'
                    if mode == normal_mode else 'recovery_controller_exception'
                ] = failure
                failures['distributed_local_solver_statuses'] = {
                    robot_id: values.get('solver_status')
                    for robot_id, values in sorted(getattr(
                        self.controller, '_last_agent_diagnostics', {}
                    ).items())
                }
                failures[status_key] = failures[status_key] or getattr(
                    self.controller, '_last_solver_status', 'exception'
                )
                selected_edges = selected_edges or getattr(
                    self.controller, '_last_selected_edges', ()
                )
                self.latest_rejected_result = locals().get('result')

        SwarmCoordinator._note_fallback_attempt(
            self, 'deterministic_recovery'
        )
        points, canonical_edges = deterministic_connectivity_setpoints(
            self.config, snapshot, selected_edges
        )
        if not canonical_edges:
            from .controllers.convex import _tree_edges
            selected_edges = _tree_edges(snapshot)
            failures['selected_edges_replaced'] = True
            points, canonical_edges = deterministic_connectivity_setpoints(
                self.config, snapshot, selected_edges
            )
        selected_edges = canonical_edges
        deterministic = ControllerResult(
            setpoints=points,
            predicted_paths={
                key: (snapshot.robots[key].position, point)
                for key, point in points.items()
            },
            selected_edges=selected_edges,
            solver_status='deterministic_recovery',
            diagnostic='Optimization stages failed.',
            created_at=time.monotonic(),
            target_epoch=snapshot.target_epoch,
            optimization_mode='deterministic_recovery',
            controller_diagnostics={
                'hard_link_limit_m': optimization_hard_link_limit(self.config)
            },
        )
        try:
            deterministic = SwarmCoordinator._process_candidate(
                self, snapshot, deterministic
            )
            if deterministic.collision_repair.get(
                'maximum_link_violation_m', 0.0
            ) > 1e-6:
                raise SafetyViolation(
                    'Deterministic recovery retains a link violation.'
                )
            self.latest_rejected_result = None
            return SwarmCoordinator._finish_outcome(
                self,
                deterministic, True, True, True,
                'deterministic_recovery', failures,
            )
        except Exception as error:
            failures['deterministic_failure_reason'] = str(error)
            self.latest_rejected_result = deterministic

        SwarmCoordinator._note_fallback_attempt(self, 'safe_hold')
        points = {
            robot_id: snapshot.robots[robot_id].position
            for robot_id in sorted(snapshot.robots)
        }
        hold = ControllerResult(
            setpoints=points,
            predicted_paths={key: (point, point) for key, point in points.items()},
            selected_edges=selected_edges,
            solver_status='safe_hold',
            diagnostic='Optimization and deterministic recovery failed.',
            created_at=time.monotonic(),
            target_epoch=snapshot.target_epoch,
            optimization_mode='safe_hold',
        )
        try:
            if not complete_finite_mapping(snapshot, hold):
                raise SafetyViolation('Safe hold is incomplete or non-finite.')
            validation = validate_controller_result(
                self.config, snapshot, hold, time.monotonic()
            )
            self.latest_collision_events = [] if validation is True else validation
            self.latest_rejected_result = None
            return SwarmCoordinator._finish_outcome(
                self,
                hold, True, True, True, 'safe_hold', failures
            )
        except Exception as error:
            failures['safe_hold_failure_reason'] = str(error)
            self.latest_rejected_result = hold
            SwarmCoordinator._note_fallback_attempt(
                self, 'non_dispatching_safe_hold'
            )
            return SwarmCoordinator._finish_outcome(
                self,
                replace(hold, optimization_mode='non_dispatching_safe_hold'),
                False,
                complete_finite_mapping(snapshot, hold),
                False,
                'non_dispatching_safe_hold',
                failures,
            )

    def _control_cycle(self):
        try:
            snapshot = self._snapshot()
        except SnapshotUnavailableError as error:
            self.latest_stop_reason = str(error)
            SwarmCoordinator._finish_outcome(
                self,
                None, False, False, False, 'pose_unavailable',
                {'pose_failure_reason': str(error)},
            )
            self._publish_controller_telemetry(
                None, None, 'pose_unavailable'
            )
            self._publish_diagnostics()
            return
        compute_reason = (
            SwarmCoordinator._computation_reason(self, snapshot)
            if hasattr(self, 'config') and hasattr(snapshot, 'station')
            else 'startup'
        )
        try:
            if compute_reason is None:
                outcome = self.latest_execution_outcome
                if outcome is None:
                    raise RuntimeError('Static controller has no cached result.')
            else:
                self._last_compute_reason = compute_reason
                self._controller_compute_count = getattr(
                    self, '_controller_compute_count', 0
                ) + 1
                outcome = self._compute_valid_result(snapshot)
        except (
            SafetyViolation,
            ControllerUnavailableError,
            RuntimeError,
            ValueError,
        ) as error:
            self.latest_stop_reason = str(error)
            outcome = SwarmCoordinator._finish_outcome(
                self,
                self.latest_rejected_result,
                False,
                False,
                False,
                'controller_error',
                {'controller_exception': SwarmCoordinator._failure(error)},
            )
            rejected = self.latest_rejected_result
            self._publish_controller_telemetry(
                snapshot,
                rejected,
                'rejected' if rejected is not None else 'faulted',
            )
            self._publish_diagnostics()
            return
        result = outcome.result
        # Clear transient pre-dispatch warnings after a valid cycle. Once any
        # rover has been commanded, faults stay latched until process restart.
        if (
            not self.dispatcher.faulted and outcome.dispatch_allowed
        ):
            self.latest_stop_reason = ''

        self.latest_snapshot = snapshot
        if compute_reason is not None:
            self.latest_result = result
        if (
            not self.dispatcher.faulted
            and outcome.dispatch_allowed
            and outcome.complete_command_set_generated
            and outcome.final_command_set_passed_validation
        ):
            if compute_reason is not None:
                try:
                    self.dispatcher.update_pending(
                        result.setpoints,
                        getattr(result, 'target_epoch', 0),
                        objective_revision=getattr(
                            self, '_objective_revision', 0
                        ),
                    )
                except TypeError:  # compatibility with narrow test adapters
                    self.dispatcher.update_pending(result.setpoints)
            self._dispatch(snapshot)
            activation_failure = getattr(
                self.dispatcher, 'last_activation_failure', ''
            )
            if activation_failure:
                self.latest_stop_reason = activation_failure
                outcome = SwarmCoordinator._finish_outcome(
                    self, result, False, True, False,
                    'non_dispatching_safe_hold',
                    {
                        'outgoing_waypoint_separation_failed':
                        activation_failure,
                    },
                )
        elif (
            not self.dispatcher.faulted
            and getattr(self.dispatcher, 'last_activation_failure', '')
        ):
            # A rejected new endpoint must not interrupt reliable refreshes of
            # unrelated commands that were already active.
            self._dispatch(snapshot)
        if hasattr(self, '_publish_target_state'):
            self._publish_target_state(snapshot)
        self._publish_controller_telemetry(
            snapshot,
            result,
            (
                'valid'
                if outcome.dispatch_allowed else outcome.controller_mode
            ),
        )
        if result is not None:
            self._publish_visualization(snapshot, result)
        self._publish_diagnostics()

    def _dispatch(self, snapshot):
        actions = self.dispatcher.tick(
            snapshot,
            time.monotonic(),
            commands_enabled=not self.config.safety.dry_run,
        )
        for action in actions:
            SwarmCoordinator._publish_dispatch_action(self, action)

    def _publish_dispatch_action(self, action):
        message = PointStamped()
        message.header.stamp.sec = int(action.token[0])
        message.header.stamp.nanosec = int(action.token[1])
        message.header.frame_id = self.config.frame_id
        message.point.x, message.point.y = action.point
        self.waypoint_publishers[action.robot_id].publish(message)
        self.latest_handoff = '%s robot=%s point=(%.3f, %.3f) token=%d.%09d' % (
            action.kind, action.robot_id, action.point[0], action.point[1],
            action.token[0], action.token[1],
        )

    def _acknowledgement(self, robot_id, message):
        token = (int(message.header.stamp.sec), int(message.header.stamp.nanosec))
        actions = self.dispatcher.acknowledge(
            robot_id,
            str(message.header.frame_id).strip(),
            token,
            (message.point.x, message.point.y),
            time.monotonic(),
            self.config.frame_id,
            measured_position=(
                self.latest_snapshot.robots[robot_id].position
                if self.latest_snapshot is not None and
                robot_id in self.latest_snapshot.robots else None
            ),
        )
        activation_failure = getattr(
            self.dispatcher, 'last_activation_failure', ''
        )
        if activation_failure:
            self.latest_stop_reason = activation_failure
            SwarmCoordinator._finish_outcome(
                self, self.latest_result, False, True, False,
                'non_dispatching_safe_hold',
                {
                    'outgoing_waypoint_separation_failed':
                    activation_failure,
                },
            )
        if not actions:
            state = self.dispatcher.states[robot_id]
            if state.last_acknowledged_token != token:
                now = time.monotonic()
                previous = self._ack_warning_times.get(robot_id)
                if previous is None or now - previous >= 2.0:
                    self._ack_warning_times[robot_id] = now
                    self.get_logger().warn(
                        'Ignored unmatched/stale waypoint acknowledgement '
                        'robot=%s token=%d.%09d.'
                        % (robot_id, token[0], token[1])
                    )
            return
        for action in actions:
            self._publish_dispatch_action(action)

    def _waypoint_failed(self, robot_id, message):
        token = (int(message.header.stamp.sec), int(message.header.stamp.nanosec))
        matched = self.dispatcher.fail(
            robot_id,
            str(message.header.frame_id).strip(),
            token,
            (message.point.x, message.point.y),
            time.monotonic(),
            self.config.frame_id,
        )
        if matched:
            self.latest_handoff = (
                'navigation_stalled robot=%s token=%d.%09d' % (
                    robot_id, token[0], token[1]
                )
            )
            self.get_logger().error(self.latest_handoff)
        else:
            self.get_logger().warn(
                'Ignored unmatched/stale waypoint_failed robot=%s '
                'token=%d.%09d.' % (robot_id, token[0], token[1])
            )

    def _onboard_health(self, robot_id, message):
        try:
            self.onboard_health[robot_id] = json.loads(message.data)
        except (ValueError, TypeError):
            self.onboard_health[robot_id] = {
                'state': 'degraded', 'restart_reason': 'malformed_health'
            }

    def _publish_target_state(self, snapshot):
        self._publish_target_values(snapshot.targets.values(), self._target_state)

    def _publish_target_values(self, targets, state):
        message = String()
        message.data = canonical_json({
            'schema_version': 1,
            **state,
            'targets': {
                target.target_id: {
                    'position': list(target.position),
                    'weight': target.weight,
                    'is_priority': target.is_priority,
                }
                for target in targets
            },
        })
        self.target_state_publisher.publish(message)

    def _stop_trial(self, reason):
        self.latest_stop_reason = str(reason)
        self.dispatcher.mark_fault(reason)
        if not self.config.safety.dry_run and not self._end_sent_for_stop:
            for robot_id in self.dispatcher.commanded_robot_ids:
                self.end_trial_publishers[robot_id].publish(Empty())
            self._end_sent_for_stop = True
        self.dispatcher.stop()
        self.get_logger().error('Swarm trial stopped: %s' % reason)

    def cleanup(self):
        if self._cleanup_complete:
            return ()
        self._stop_trial('coordinator shutdown')
        self._cleanup_complete = True
        return self.dispatcher.commanded_robot_ids

    def _publish_diagnostics(self):
        now = time.monotonic()
        ages = self.aggregator.pose_ages(now)
        dispatch_observability = self.dispatcher.observability(now)
        diagnostic_result = self.latest_rejected_result or self.latest_result
        result_state = (
            'rejected' if self.latest_rejected_result is not None
            else ('valid' if self.latest_result is not None else 'none')
        )
        outcome = getattr(self, 'latest_execution_outcome', None)
        values = {
            'algorithm': self.config.controller.algorithm,
            'dry_run': str(self.config.safety.dry_run),
            'dispatch_state': (
                'dry_run' if self.config.safety.dry_run else
                ('faulted' if self.dispatcher.faulted else
                 ('inhibited' if outcome and not outcome.dispatch_allowed
                  else 'automatic'))
            ),
            'controller_result_state': result_state,
            'solver_status': (
                diagnostic_result.solver_status if diagnostic_result else 'none'
            ),
            'solve_duration_sec': (
                '%.6f' % diagnostic_result.solve_duration_sec
                if diagnostic_result else 'nan'
            ),
            'stop_reason': self.latest_stop_reason,
            'commanded_robots': ','.join(self.dispatcher.commanded_robot_ids),
            'latest_handoff': self.latest_handoff,
            'selected_edges': (
                str(diagnostic_result.selected_edges)
                if diagnostic_result else 'none'
            ),
            'controller_mode': (
                outcome.controller_mode if outcome else 'none'
            ),
            'dispatch_allowed': (
                outcome.dispatch_allowed if outcome else False
            ),
            'complete_command_set_generated': (
                outcome.complete_command_set_generated if outcome else False
            ),
            'final_command_set_passed_validation': (
                outcome.final_command_set_passed_validation
                if outcome else False
            ),
            'consecutive_recovery_cycles': (
                outcome.consecutive_recovery_cycles if outcome else 0
            ),
            'fallback_counters': (
                dict(outcome.fallback_counters) if outcome else {}
            ),
            'computation_reason': getattr(
                self, '_last_compute_reason', 'not_computed'
            ),
            'objective_revision': getattr(self, '_objective_revision', 0),
            'controller_compute_count': getattr(
                self, '_controller_compute_count', 0
            ),
        }
        for robot_id, age in ages.items():
            values['pose_age_' + robot_id] = (
                'missing' if age is None else '%.3f' % age
            )
            state = self.dispatcher.states[robot_id]
            values['active_' + robot_id] = str(state.active_waypoint)
            values['pending_' + robot_id] = str(state.pending_waypoint)
            dispatch = dispatch_observability[robot_id]
            for field in (
                'active_waypoint_age_sec',
                'last_publication_monotonic_sec',
                'last_publication_age_sec',
                'refresh_count',
                'active_waypoint_overdue',
                'active_token',
                'last_acknowledged_token',
                'acknowledgement_count',
                'last_acknowledgement_monotonic_sec',
                'last_acknowledgement_age_sec',
                'unmatched_acknowledgement_count',
                'handoff_cause',
                'active_target_epoch',
                'pending_target_epoch',
                'active_objective_revision',
                'pending_objective_revision',
            ):
                values[field + '_' + robot_id] = str(dispatch[field])
            health = getattr(self, 'onboard_health', {}).get(robot_id)
            values['onboard_health_' + robot_id] = (
                'missing' if health is None else health.get('state', 'unknown')
            )
            values['onboard_restart_reason_' + robot_id] = (
                '' if health is None else health.get('restart_reason', '')
            )
            values['waypoint_publisher_count_' + robot_id] = (
                'unknown' if health is None else
                health.get('waypoint_publisher_count', 'unknown')
            )
            values['controller_navigation_state_' + robot_id] = (
                'unknown' if health is None else
                health.get('controller_navigation_state', 'unknown')
            )
            serial = {} if health is None else health.get(
                'serial_counters', {}
            )
            values['serial_consecutive_failures_' + robot_id] = serial.get(
                'consecutive_failures', 'unknown'
            )
        if self.latest_snapshot is not None:
            values['priority_target_id'] = str(
                self.latest_snapshot.priority_target_id
            )
            values['target_epoch'] = str(self.latest_snapshot.target_epoch)
            values['target_selection_seed'] = str(
                self.latest_snapshot.target_selection_seed
            )
            robot_points = [
                state.position for state in self.latest_snapshot.robots.values()
            ]
            values['minimum_pairwise_distance_m'] = str(
                minimum_pairwise_distance(robot_points)
            )
            node_positions = {
                self.latest_snapshot.station.station_id:
                self.latest_snapshot.station.position,
                **{
                    key: state.position
                    for key, state in self.latest_snapshot.robots.items()
                },
            }
            values['lambda_2'] = str(algebraic_connectivity(
                node_positions, self.config.communication.maximum_range_m
            ))
        status = DiagnosticStatus()
        status.name = 'waverover_swarm/controller'
        status.hardware_id = 'operator_pc'
        overdue = [
            robot_id for robot_id, dispatch in dispatch_observability.items()
            if dispatch['active_waypoint_overdue']
        ]
        status.level = DiagnosticStatus.WARN if (
            self.latest_stop_reason or overdue
        ) else DiagnosticStatus.OK
        status.message = (
            self.latest_stop_reason or
            ('active waypoint overdue: ' + ','.join(overdue)
             if overdue else 'cycle valid')
        )
        status.values = [
            KeyValue(key=key, value=str(value))
            for key, value in sorted(values.items())
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_publisher.publish(message)

    def _publish_controller_telemetry(self, snapshot, result, result_state):
        """Publish analysis telemetry without participating in safety logic."""
        try:
            now = time.monotonic()
            pose_ages = self.aggregator.pose_ages(now)
            active = {
                robot_id: state.active_waypoint
                for robot_id, state in self.dispatcher.states.items()
            }
            pending = {
                robot_id: state.pending_waypoint
                for robot_id, state in self.dispatcher.states.items()
            }
            dispatch_observability = self.dispatcher.observability(now)
            payload = build_controller_telemetry(
                self.config,
                snapshot,
                result,
                result_state,
                self.get_clock().now().to_msg(),
                not self.config.safety.dry_run
                and not self.dispatcher.faulted
                and (
                    self.latest_execution_outcome is None
                    or self.latest_execution_outcome.dispatch_allowed
                ),
                self.latest_stop_reason,
                pose_ages,
                active,
                pending,
                dispatch_observability,
                self.latest_handoff,
                self.latest_execution_outcome,
                computation_reason=getattr(
                    self, '_last_compute_reason', 'not_computed'
                ),
                objective_revision=getattr(self, '_objective_revision', 0),
                controller_compute_count=getattr(
                    self, '_controller_compute_count', 0
                ),
                activation_repair=(
                    asdict(self.dispatcher.last_activation_report)
                    if getattr(
                        self.dispatcher, 'last_activation_report', None
                    ) is not None
                    else {}
                ),
            )
            message = String()
            message.data = canonical_json(payload)
            self.telemetry_publisher.publish(message)
        except Exception as error:  # analysis output must not affect safety
            self.get_logger().error(
                'Controller telemetry publication failed: %s' % error
            )

    def _publish_visualization(self, snapshot, result):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        points = [
            ('station', snapshot.station.station_id, snapshot.station.position, 0),
        ]
        points.extend(
            ('target', target.target_id, target.position, 1)
            for target in snapshot.targets.values()
        )
        points.extend(
            ('setpoint', robot_id, point, 2)
            for robot_id, point in result.setpoints.items()
        )
        marker_id = 0
        for namespace, label, point, marker_type in points:
            marker = Marker()
            marker.header.frame_id = self.config.frame_id
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE if marker_type != 0 else Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.12
            marker.color.a = 1.0
            marker.color.r = 1.0 if namespace == 'target' else 0.1
            marker.color.g = 1.0 if namespace == 'setpoint' else 0.2
            marker.color.b = 1.0 if namespace == 'station' else 0.2
            marker.text = label
            markers.markers.append(marker)
            marker_id += 1
        edge_marker = Marker()
        edge_marker.header.frame_id = self.config.frame_id
        edge_marker.header.stamp = stamp
        edge_marker.ns = 'selected_edges'
        edge_marker.id = marker_id
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.025
        edge_marker.color.a = 1.0
        edge_marker.color.r = 1.0
        node_positions = {
            snapshot.station.station_id: snapshot.station.position,
            **{key: state.position for key, state in snapshot.robots.items()},
        }
        for first, second in result.selected_edges:
            for node_id in (first, second):
                point = Point()
                point.x, point.y = node_positions[node_id]
                edge_marker.points.append(point)
        markers.markers.append(edge_marker)
        self.markers_publisher.publish(markers)

        for robot_id, points_path in result.predicted_paths.items():
            path = Path()
            path.header.frame_id = self.config.frame_id
            path.header.stamp = stamp
            for x, y in points_path:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.path_publishers[robot_id].publish(path)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SwarmCoordinator()
    previous_handlers = {}

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        commanded = node.cleanup()
        if commanded and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
