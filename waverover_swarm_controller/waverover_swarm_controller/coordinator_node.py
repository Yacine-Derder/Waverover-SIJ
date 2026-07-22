"""ROS boundary for the operator-PC swarm coordinator."""

from dataclasses import replace
import json
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
    ControllerUnavailableError,
    repair_controller_result,
)
from .metrics import algebraic_connectivity, minimum_pairwise_distance
from .models import ControllerResult
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
        algorithm = str(self.declare_parameter('algorithm', 'heuristic').value)
        dry_run = bool(self.declare_parameter('dry_run', True).value)
        if not config_file:
            raise ConfigError('config_file is required.')
        self.config = load_experiment(
            config_file,
            algorithm_override=algorithm,
            dry_run_override=dry_run,
        )
        self.controller = controller_from_config(self.config)
        self.target_manager = TargetManager(
            self.config.targets, self.config.target_dynamics,
            start_time=time.monotonic(),
        )
        self.dispatcher = WaypointDispatcher(
            self.config.robot_ids, self.config.waypoint_dispatch
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

    def _compute_valid_result(self, snapshot):
        optimization = self.config.controller.algorithm in (
            'convex', 'mpc_centralized', 'mpc_distributed'
        )
        self._optimization_dispatch_allowed = True
        attempts = [('normal', self.controller.compute)]
        recovery = getattr(self.controller, 'compute_recovery', None)
        if optimization and recovery is not None:
            attempts.append(('recovery', recovery))
        errors = []
        for label, operation in attempts:
            try:
                result = operation(snapshot)
                result = SwarmCoordinator._repair_result(self, snapshot, result)
                collision_validation = validate_controller_result(
                    self.config, snapshot, result, time.monotonic()
                )
                self.latest_collision_events = (
                    [] if collision_validation is True else collision_validation
                )
                self.latest_rejected_result = None
                return result
            except Exception as error:  # solver/controller failures are recoverable
                errors.append('%s: %s' % (label, error))
                self.latest_rejected_result = locals().get('result')
                if not optimization:
                    raise
        # Deterministic connectivity recovery moves each child toward a stable
        # station-rooted nearest tree, without exceeding the physical step.
        for mode in ('deterministic_recovery', 'safe_hold'):
            points = {
                robot_id: snapshot.robots[robot_id].position
                for robot_id in sorted(snapshot.robots)
            }
            edges = ()
            if mode == 'deterministic_recovery':
                from .controllers.convex import _tree_edges
                import math

                edges = _tree_edges(snapshot)
                maximum_link = (
                    self.config.communication.maximum_range_m
                    - self.config.vehicle.turn_radius_m
                )
                for parent, robot_id in edges:
                    parent_point = (
                        snapshot.station.position
                        if parent == snapshot.station.station_id
                        else points[parent]
                    )
                    current = snapshot.robots[robot_id].position
                    distance = math.dist(current, parent_point)
                    if distance > maximum_link and distance > 1e-12:
                        travel = min(
                            self.config.controller.mpc_max_step_m,
                            distance - maximum_link,
                        )
                        points[robot_id] = (
                            current[0] + travel
                            * (parent_point[0] - current[0]) / distance,
                            current[1] + travel
                            * (parent_point[1] - current[1]) / distance,
                        )
            result = ControllerResult(
                setpoints=points,
                predicted_paths={
                    key: (point, point) for key, point in points.items()
                },
                selected_edges=edges,
                solver_status=mode,
                diagnostic='; '.join(errors),
                created_at=time.monotonic(),
                target_epoch=snapshot.target_epoch,
                optimization_mode=mode,
            )
            try:
                if mode != 'safe_hold':
                    result = SwarmCoordinator._repair_result(
                        self, snapshot, result
                    )
                collision_validation = validate_controller_result(
                    self.config, snapshot, result, time.monotonic()
                )
                if (
                    mode == 'deterministic_recovery'
                    and result.collision_repair.get(
                        'maximum_link_violation_m', 0.0
                    ) > 1e-6
                ):
                    raise SafetyViolation(
                        'Deterministic recovery cannot restore every link '
                        'within one physical step.'
                    )
                self.latest_collision_events = (
                    [] if collision_validation is True else collision_validation
                )
                self.latest_rejected_result = None
                return result
            except Exception as error:
                errors.append('%s: %s' % (mode, error))
        # Keep the process and dispatcher healthy, but do not enqueue a hold
        # which failed pre-dispatch validation.
        self._optimization_dispatch_allowed = False
        self.latest_stop_reason = '; '.join(errors)
        self.latest_rejected_result = result
        return replace(result, diagnostic=self.latest_stop_reason)

    def _repair_result(self, snapshot, result):
        active = {
            robot_id: state.active_waypoint
            for robot_id, state in getattr(
                self.dispatcher, 'states', {}
            ).items()
            if state.active_waypoint is not None
        }
        output = repair_controller_result(
            self.config, snapshot, result, active=active
        )
        metadata = dict(output.collision_repair)
        metadata['controller_output_repair'] = dict(result.collision_repair)
        metadata['collision_events'] = tuple(getattr(
            self, 'latest_collision_events', ()
        ))
        metadata['predicted_paths_after_first_step'] = 'pre_repair'
        if metadata['residual_violation_m'] > 1e-6:
            logger = self.get_logger() if hasattr(self, 'get_logger') else None
            if logger is not None:
                logger.warn(
                    'Best-effort waypoint separation has %.3f m residual; '
                    'continuing with finite geofence-valid least-violating '
                    'output.' % metadata['residual_violation_m']
                )
        return replace(
            output,
            collision_repair=metadata,
        )

    def _control_cycle(self):
        try:
            snapshot = self._snapshot()
        except SnapshotUnavailableError as error:
            self.latest_stop_reason = str(error)
            if self.dispatcher.commanded_robot_ids:
                self._stop_trial('stale/incomplete required pose: %s' % error)
            self._publish_controller_telemetry(None, None, 'faulted')
            self._publish_diagnostics()
            return
        try:
            result = self._compute_valid_result(snapshot)
        except (
            SafetyViolation,
            ControllerUnavailableError,
            RuntimeError,
            ValueError,
        ) as error:
            self.latest_stop_reason = str(error)
            optimization = self.config.controller.algorithm in (
                'convex', 'mpc_centralized', 'mpc_distributed'
            )
            if self.dispatcher.commanded_robot_ids and not optimization:
                self._stop_trial('invalid controller cycle: %s' % error)
            rejected = self.latest_rejected_result
            self._publish_controller_telemetry(
                snapshot,
                rejected,
                'rejected' if rejected is not None else 'faulted',
            )
            self._publish_diagnostics()
            return
        # Clear transient pre-dispatch warnings after a valid cycle. Once any
        # rover has been commanded, faults stay latched until process restart.
        if (
            not self.dispatcher.faulted
            and getattr(self, '_optimization_dispatch_allowed', True)
        ):
            self.latest_stop_reason = ''

        self.latest_snapshot = snapshot
        self.latest_result = result
        if not self.dispatcher.faulted:
            try:
                self.dispatcher.update_pending(
                    result.setpoints, getattr(result, 'target_epoch', 0)
                )
            except TypeError:  # compatibility with narrow test/legacy adapters
                self.dispatcher.update_pending(result.setpoints)
            self._dispatch(snapshot)
        if hasattr(self, '_publish_target_state'):
            self._publish_target_state(snapshot)
        self._publish_controller_telemetry(
            snapshot,
            result,
            (
                'valid'
                if getattr(self, '_optimization_dispatch_allowed', True)
                else 'non_dispatching_safe_hold'
            ),
        )
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
        values = {
            'algorithm': self.config.controller.algorithm,
            'dry_run': str(self.config.safety.dry_run),
            'dispatch_state': (
                'dry_run' if self.config.safety.dry_run else
                ('faulted' if self.dispatcher.faulted else 'automatic')
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
                not self.config.safety.dry_run and not self.dispatcher.faulted,
                self.latest_stop_reason,
                pose_ages,
                active,
                pending,
                dispatch_observability,
                self.latest_handoff,
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
