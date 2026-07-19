"""Synchronized synthetic MCS poses for operator-PC development only."""

from dataclasses import dataclass
import json
import math
import signal

from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from waverover.stack_config import load_stack_config, mcs_pose_topic

from .config import ConfigError, load_experiment
from .metrics import (
    algebraic_connectivity,
    connected_components,
    minimum_pairwise_with_ids,
    weighted_algebraic_connectivity,
)
from .synthetic_motion import SyntheticTrajectory, yaw_quaternion


@dataclass(frozen=True)
class FormationValidation:
    minimum_separation_m: float
    minimum_separation_pair: object
    binary_lambda_2: float
    weighted_lambda_2: float
    connected_components: int
    station_reachable_rovers: int
    disconnected: bool

    @property
    def algebraic_connectivity(self):
        """Backward-compatible name used by existing callers."""
        return self.binary_lambda_2


def validated_parameters(rate_hz, radius_m, angle_offset_rad, yaw_rad):
    """Normalize and validate synthetic publisher numeric parameters."""
    values = []
    for name, value in (
        ('rate_hz', rate_hz),
        ('radius_m', radius_m),
        ('angle_offset_rad', angle_offset_rad),
        ('yaw_rad', yaw_rad),
    ):
        if isinstance(value, bool):
            raise ConfigError('%s must be numeric.' % name)
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ConfigError('%s must be numeric.' % name) from error
        if not math.isfinite(number):
            raise ConfigError('%s must be finite.' % name)
        values.append(number)
    rate, radius, offset, yaw = values
    if rate <= 0.0:
        raise ConfigError('rate_hz must be positive.')
    if radius < 0.0:
        raise ConfigError('radius_m must be nonnegative.')
    return rate, radius, offset, yaw


def resolve_initial_radius(config, radius_override=''):
    """Apply CLI override, YAML value, then legacy 0.5 m precedence."""
    if radius_override is None or str(radius_override).strip() == '':
        return float(getattr(config.synthetic_mcs, 'initial_radius_m', 0.5))
    try:
        radius = float(radius_override)
    except (TypeError, ValueError) as error:
        raise ConfigError('radius_m must be numeric or empty.') from error
    if not math.isfinite(radius) or radius < 0.0:
        raise ConfigError('radius_m must be finite and nonnegative.')
    return radius


def generate_formation(robot_ids, station_position, radius_m, angle_offset_rad=0.0):
    """Build deterministic, ID-sorted planar positions around a station."""
    ordered_ids = tuple(sorted(str(robot_id) for robot_id in robot_ids))
    if not ordered_ids:
        raise ConfigError('Synthetic formation requires at least one robot.')
    center_x, center_y = station_position
    if len(ordered_ids) == 1:
        return {ordered_ids[0]: (float(center_x), float(center_y))}
    return {
        robot_id: (
            float(center_x) + radius_m * math.cos(
                angle_offset_rad + 2.0 * math.pi * index / len(ordered_ids)
            ),
            float(center_y) + radius_m * math.sin(
                angle_offset_rad + 2.0 * math.pi * index / len(ordered_ids)
            ),
        )
        for index, robot_id in enumerate(ordered_ids)
    }


def _is_connected(node_positions, maximum_range_m):
    node_ids = tuple(sorted(node_positions))
    visited = {node_ids[0]}
    pending = [node_ids[0]]
    while pending:
        first = pending.pop()
        for second in node_ids:
            if second not in visited and math.dist(
                node_positions[first], node_positions[second]
            ) <= maximum_range_m:
                visited.add(second)
                pending.append(second)
    return len(visited) == len(node_ids)


def validate_formation(config, positions, connectivity_policy=None):
    """Reject unsafe or configuration-inconsistent synthetic formations."""
    expected = set(config.robot_ids)
    received = set(positions)
    if received != expected:
        raise ConfigError(
            'Formation IDs mismatch; missing=%s extra=%s.'
            % (sorted(expected - received), sorted(received - expected))
        )
    for robot_id, point in positions.items():
        if len(point) != 2 or not all(math.isfinite(float(value)) for value in point):
            raise ConfigError('Formation position for %s must be finite.' % robot_id)
        if not config.safety.geofence.contains(point):
            raise ConfigError('Formation position for %s is outside the geofence.' % robot_id)
    separation, first, second = minimum_pairwise_with_ids(positions)
    if separation < config.safety.minimum_separation_m:
        raise ConfigError(
            'Formation minimum separation between %s and %s is %.3f m, '
            'below %.3f m.'
            % (first, second, separation, config.safety.minimum_separation_m)
        )
    nodes = {config.station.station_id: config.station.position, **positions}
    components = connected_components(
        nodes, config.communication.maximum_range_m
    )
    disconnected = len(components) > 1
    policy = connectivity_policy or config.synthetic_mcs.connectivity_policy
    if policy == 'enforce' and disconnected:
        raise ConfigError(
            'Formation graph including station is disconnected at %.3f m.'
            % config.communication.maximum_range_m
        )
    connectivity = algebraic_connectivity(
        nodes, config.communication.maximum_range_m
    )
    weighted = weighted_algebraic_connectivity(
        nodes,
        config.communication.ideal_range_m,
        config.communication.maximum_range_m,
        config.analysis.connectivity_alpha,
    )
    station_component = next((
        component for component in components
        if config.station.station_id in component
    ), ())
    return FormationValidation(
        separation,
        [first, second] if first is not None else None,
        connectivity,
        weighted,
        len(components),
        max(0, len(station_component) - 1),
        disconnected,
    )


class SyntheticMCS(Node):
    """Publish poses only; this node has no command-side interfaces."""

    def __init__(self):
        super().__init__('synthetic_mcs')
        config_file = str(self.declare_parameter('config_file', '').value).strip()
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        radius_override = self.declare_parameter('radius_m', '').value
        angle_offset = self.declare_parameter('angle_offset_rad', 0.0).value
        yaw_rad = self.declare_parameter('yaw_rad', 0.0).value
        seed_override = str(
            self.declare_parameter('seed_override', '').value
        ).strip()
        if not config_file:
            raise ConfigError('config_file is required.')
        self.config = load_experiment(config_file)
        radius_m = resolve_initial_radius(self.config, radius_override)
        rate_hz, radius_m, angle_offset, yaw_rad = validated_parameters(
            rate_hz, radius_m, angle_offset, yaw_rad
        )
        stack_config = load_stack_config(require_identity=False)
        initial_positions = generate_formation(
            self.config.robot_ids,
            self.config.station.position,
            radius_m,
            angle_offset,
        )
        validation = validate_formation(self.config, initial_positions)
        try:
            selected_seed = None if not seed_override else int(seed_override)
        except ValueError as error:
            raise ConfigError('seed_override must be an integer or empty.') from error
        self.trajectory = SyntheticTrajectory(
            self.config,
            initial_positions,
            rate_hz,
            initial_yaw=yaw_rad,
            seed=selected_seed,
            initial_radius_m=radius_m,
        )
        self.trajectory.last_true_validation = validation
        self.positions = dict(initial_positions)
        self._publish_count = 0
        self._failed = False
        mcs_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(stack_config['mcs']['qos_depth']),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        metadata_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publishers_by_id = {
            robot_id: self.create_publisher(
                PoseStamped, mcs_pose_topic(stack_config, robot_id), mcs_qos
            )
            for robot_id in sorted(initial_positions)
        }
        self.ground_truth_publishers = {
            robot_id: self.create_publisher(
                PoseStamped,
                'synthetic/ground_truth/waverover_' + robot_id,
                reliable,
            )
            for robot_id in sorted(initial_positions)
        }
        self.motion_publishers = {
            robot_id: self.create_publisher(
                TwistStamped,
                'synthetic/motion/waverover_' + robot_id,
                reliable,
            )
            for robot_id in sorted(initial_positions)
        }
        self.metadata_publisher = self.create_publisher(
            String, 'synthetic/metadata', metadata_qos
        )
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_poses)
        self._publish_metadata('running')
        self.get_logger().info(
            'Synthetic MCS: robots=%d rate_hz=%.3f mode=%s seed=%d '
            'minimum_separation_m=%s algebraic_connectivity=%.6f'
            % (
                len(self.positions),
                rate_hz,
                self.config.synthetic_mcs.mode,
                self.trajectory.actual_seed,
                ('inf' if math.isinf(validation.minimum_separation_m) else
                 '%.6f' % validation.minimum_separation_m),
                validation.algebraic_connectivity,
            )
        )
        for robot_id, point in self.positions.items():
            self.get_logger().info(
                'robot=%s topic=%s position=(%.6f, %.6f)'
                % (
                    robot_id,
                    mcs_pose_topic(stack_config, robot_id),
                    point[0],
                    point[1],
                )
            )

    def _publish_metadata(self, state, error=''):
        metadata = self.trajectory.metadata()
        metadata.update({
            'state': state,
            'error': str(error),
            'elapsed_sec': self.trajectory.elapsed,
        })
        message = String()
        message.data = json.dumps(
            metadata, sort_keys=True, separators=(',', ':')
        )
        self.metadata_publisher.publish(message)

    @staticmethod
    def _pose_message(stamp, frame_id, point, yaw):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.pose.position.x = point[0]
        message.pose.position.y = point[1]
        qx, qy, qz, qw = yaw_quaternion(yaw)
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        return message

    def _fail_closed(self, error):
        if self._failed:
            return
        self._failed = True
        self.timer.cancel()
        self._publish_metadata('failed', error)
        self.get_logger().error(
            'Synthetic MCS stopped publishing: %s' % error
        )

    def _publish_poses(self):
        if self._failed:
            return
        if self.trajectory.elapsed >= self.config.synthetic_mcs.duration_sec:
            self.timer.cancel()
            self._publish_metadata('completed')
            self.get_logger().info('Synthetic MCS duration completed.')
            return
        previous_segment_count = len(self.trajectory.generated_segments)
        try:
            result = self.trajectory.step(
                lambda values: validate_formation(self.config, values)
            )
            observed, observed_headings = self.trajectory.observed_formation(
                lambda values: validate_formation(self.config, values)
            )
        except (ConfigError, ValueError) as error:
            self._fail_closed(error)
            return
        self.positions = dict(result.positions)
        stamp = self.get_clock().now().to_msg()
        for robot_id in sorted(self.positions):
            self.ground_truth_publishers[robot_id].publish(self._pose_message(
                stamp,
                self.config.frame_id,
                self.positions[robot_id],
                result.headings[robot_id],
            ))
            self.publishers_by_id[robot_id].publish(self._pose_message(
                stamp,
                self.config.frame_id,
                observed[robot_id],
                observed_headings[robot_id],
            ))
            motion = TwistStamped()
            motion.header.stamp = stamp
            motion.header.frame_id = self.config.frame_id
            motion.twist.linear.x = result.speeds[robot_id]
            motion.twist.angular.z = result.yaw_rates[robot_id]
            self.motion_publishers[robot_id].publish(motion)
        self._publish_count += 1
        if (
            self._publish_count % max(1, int(round(1.0 / self.trajectory.timestep)))
            == 0
            or len(self.trajectory.generated_segments) != previous_segment_count
        ):
            self._publish_metadata('running')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, interrupt)
    try:
        node = SyntheticMCS()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
