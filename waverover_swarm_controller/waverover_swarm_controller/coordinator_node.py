"""ROS boundary for the operator-PC swarm coordinator."""

from dataclasses import replace
import signal
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

from .config import ConfigError, load_experiment, SUPPORTED_ALGORITHMS
from .controllers import controller_from_config
from .controllers.base import ControllerUnavailableError
from .metrics import algebraic_connectivity, minimum_pairwise_distance
from .pose_aggregation import PoseAggregator, SnapshotUnavailableError
from .safety import SafetyViolation, validate_controller_result
from .waypoint_dispatcher import WaypointDispatcher


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
        self.armed = False
        self.latest_result = None
        self.latest_snapshot = None
        self.latest_stop_reason = 'startup: dry-run, disarmed'
        self.latest_handoff = 'none'
        self._cleanup_complete = False
        self._end_sent_for_stop = False

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

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, 'diagnostics', reliable
        )
        self.markers_publisher = self.create_publisher(
            MarkerArray, 'markers', reliable
        )
        self.path_publishers = {
            robot_id: self.create_publisher(
                Path, 'predicted_path/' + robot_id, reliable
            )
            for robot_id in self.config.robot_ids
        }
        self.arm_service = self.create_service(SetBool, 'arm', self._arm_callback)
        self.add_on_set_parameters_callback(self._parameters_callback)
        self.timer = self.create_timer(
            self.config.controller.control_period_sec,
            self._control_cycle,
        )
        self.get_logger().warn(
            'PC-only coordinator algorithm=%s dry_run=%s armed=false. '
            'No rover commands are being sent.'
            % (self.config.controller.algorithm, self.config.safety.dry_run)
        )

    def _parameters_callback(self, parameters):
        for parameter in parameters:
            if parameter.name in ('config_file', 'dry_run'):
                return SetParametersResult(
                    successful=False,
                    reason='%s changes require a coordinator relaunch.'
                    % parameter.name,
                )
            if parameter.name == 'algorithm':
                if self.armed:
                    return SetParametersResult(
                        successful=False,
                        reason='Disarm before changing algorithms.',
                    )
                algorithm = str(parameter.value).strip().lower()
                if algorithm not in SUPPORTED_ALGORITHMS:
                    return SetParametersResult(
                        successful=False,
                        reason='Unknown swarm algorithm %s.' % algorithm,
                    )
                selected = replace(
                    self.config,
                    controller=replace(
                        self.config.controller,
                        algorithm=algorithm,
                    ),
                )
                self.config = selected
                self.controller = controller_from_config(selected)
                self.latest_result = None
                self.latest_stop_reason = (
                    'algorithm changed while disarmed; awaiting valid cycle'
                )
        return SetParametersResult(successful=True)

    def _snapshot(self):
        return self.aggregator.snapshot(
            self.config.targets,
            self.config.station,
        )

    def _compute_valid_result(self, snapshot):
        result = self.controller.compute(snapshot)
        validate_controller_result(
            self.config, snapshot, result, time.monotonic()
        )
        return result

    def _arm_callback(self, request, response):
        if not request.data:
            self._stop_trial('explicit disarm')
            response.success = True
            response.message = 'Disarmed; end-trial sent to commanded rovers.'
            return response
        if self.armed:
            response.success = True
            response.message = 'Already armed.'
            return response
        if self.config.safety.dry_run:
            response.success = False
            response.message = 'Cannot arm while dry_run is true.'
            return response
        available, reason = self.controller.availability()
        if not available:
            response.success = False
            response.message = reason
            return response
        try:
            snapshot = self._snapshot()
            result = self._compute_valid_result(snapshot)
        except (
            SnapshotUnavailableError,
            SafetyViolation,
            ControllerUnavailableError,
            RuntimeError,
            ValueError,
        ) as error:
            response.success = False
            response.message = 'Arming rejected: %s' % error
            return response
        self.dispatcher.rearm()
        self.dispatcher.update_pending(result.setpoints)
        self.latest_snapshot = snapshot
        self.latest_result = result
        self.armed = True
        self._end_sent_for_stop = False
        self.latest_stop_reason = ''
        self._dispatch(snapshot)
        response.success = True
        response.message = 'Armed with validated %s result.' % (
            self.config.controller.algorithm
        )
        return response

    def _control_cycle(self):
        try:
            snapshot = self._snapshot()
        except SnapshotUnavailableError as error:
            self.latest_stop_reason = str(error)
            if self.armed:
                self._stop_trial('stale/incomplete required pose: %s' % error)
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
            if self.armed:
                self._stop_trial('invalid controller cycle: %s' % error)
            self._publish_diagnostics()
            return
        self.latest_snapshot = snapshot
        self.latest_result = result
        self.dispatcher.update_pending(result.setpoints)
        if self.armed:
            self._dispatch(snapshot)
        self._publish_visualization(snapshot, result)
        self._publish_diagnostics()

    def _dispatch(self, snapshot):
        actions = self.dispatcher.tick(
            snapshot,
            time.monotonic(),
            commands_enabled=self.armed and not self.config.safety.dry_run,
        )
        for action in actions:
            if action.kind == 'fault':
                self._stop_trial(action.reason)
                return
            message = PointStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.config.frame_id
            message.point.x = action.point[0]
            message.point.y = action.point[1]
            self.waypoint_publishers[action.robot_id].publish(message)
            self.latest_handoff = 'robot=%s point=(%.3f, %.3f)' % (
                action.robot_id, action.point[0], action.point[1]
            )
            self.get_logger().info(
                'Waypoint handoff robot=%s x=%.3f y=%.3f.'
                % (action.robot_id, action.point[0], action.point[1])
            )

    def _stop_trial(self, reason):
        self.armed = False
        self.latest_stop_reason = str(reason)
        self.dispatcher.mark_fault(reason)
        if not self._end_sent_for_stop:
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
        values = {
            'algorithm': self.config.controller.algorithm,
            'dry_run': str(self.config.safety.dry_run),
            'armed': str(self.armed),
            'solver_status': (
                self.latest_result.solver_status if self.latest_result else 'none'
            ),
            'solve_duration_sec': (
                '%.6f' % self.latest_result.solve_duration_sec
                if self.latest_result else 'nan'
            ),
            'stop_reason': self.latest_stop_reason,
            'commanded_robots': ','.join(self.dispatcher.commanded_robot_ids),
            'latest_handoff': self.latest_handoff,
            'selected_edges': (
                str(self.latest_result.selected_edges)
                if self.latest_result else 'none'
            ),
        }
        for robot_id, age in ages.items():
            values['pose_age_' + robot_id] = (
                'missing' if age is None else '%.3f' % age
            )
            state = self.dispatcher.states[robot_id]
            values['active_' + robot_id] = str(state.active_waypoint)
            values['pending_' + robot_id] = str(state.pending_waypoint)
        if self.latest_snapshot is not None:
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
        status.level = (
            DiagnosticStatus.OK if not self.latest_stop_reason
            else DiagnosticStatus.WARN
        )
        status.message = self.latest_stop_reason or 'cycle valid'
        status.values = [
            KeyValue(key=key, value=str(value))
            for key, value in sorted(values.items())
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_publisher.publish(message)

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
