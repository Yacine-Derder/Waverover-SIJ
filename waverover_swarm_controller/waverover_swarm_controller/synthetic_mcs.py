"""Synchronized synthetic MCS poses for operator-PC development only."""

from dataclasses import dataclass
import math
import signal

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from waverover.stack_config import load_stack_config, mcs_pose_topic

from .config import ConfigError, load_experiment
from .metrics import algebraic_connectivity, minimum_pairwise_distance


@dataclass(frozen=True)
class FormationValidation:
    minimum_separation_m: float
    algebraic_connectivity: float


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


def validate_formation(config, positions):
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
    separation = minimum_pairwise_distance(positions.values())
    if separation < config.safety.minimum_separation_m:
        raise ConfigError(
            'Formation minimum separation %.3f m is below %.3f m.'
            % (separation, config.safety.minimum_separation_m)
        )
    nodes = {config.station.station_id: config.station.position, **positions}
    if not _is_connected(nodes, config.communication.maximum_range_m):
        raise ConfigError(
            'Formation graph including station is disconnected at %.3f m.'
            % config.communication.maximum_range_m
        )
    connectivity = algebraic_connectivity(
        nodes, config.communication.maximum_range_m
    )
    return FormationValidation(separation, connectivity)


class SyntheticMCS(Node):
    """Publish poses only; this node has no command-side interfaces."""

    def __init__(self):
        super().__init__('synthetic_mcs')
        config_file = str(self.declare_parameter('config_file', '').value).strip()
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        radius_m = self.declare_parameter('radius_m', 0.5).value
        angle_offset = self.declare_parameter('angle_offset_rad', 0.0).value
        yaw_rad = self.declare_parameter('yaw_rad', 0.0).value
        if not config_file:
            raise ConfigError('config_file is required.')
        rate_hz, radius_m, angle_offset, yaw_rad = validated_parameters(
            rate_hz, radius_m, angle_offset, yaw_rad
        )
        self.config = load_experiment(config_file)
        stack_config = load_stack_config(require_identity=False)
        self.positions = generate_formation(
            self.config.robot_ids,
            self.config.station.position,
            radius_m,
            angle_offset,
        )
        validation = validate_formation(self.config, self.positions)
        self._quaternion_z = math.sin(yaw_rad / 2.0)
        self._quaternion_w = math.cos(yaw_rad / 2.0)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(stack_config['mcs']['qos_depth']),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publishers_by_id = {
            robot_id: self.create_publisher(
                PoseStamped, mcs_pose_topic(stack_config, robot_id), qos
            )
            for robot_id in sorted(self.positions)
        }
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_poses)
        self.get_logger().info(
            'Synthetic MCS: robots=%d rate_hz=%.3f minimum_separation_m=%s '
            'algebraic_connectivity=%.6f'
            % (
                len(self.positions),
                rate_hz,
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

    def _publish_poses(self):
        stamp = self.get_clock().now().to_msg()
        for robot_id, (x, y) in self.positions.items():
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = self.config.frame_id
            message.pose.position.x = x
            message.pose.position.y = y
            message.pose.orientation.z = self._quaternion_z
            message.pose.orientation.w = self._quaternion_w
            self.publishers_by_id[robot_id].publish(message)


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
