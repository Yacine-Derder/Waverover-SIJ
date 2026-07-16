from collections import deque
from dataclasses import dataclass
import math
import time

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from waverover.stack_config import (
    load_stack_config,
    mcs_pose_topic,
    normalize_control_mode,
    normalize_pose_source,
    required,
    robot_frame,
    StackConfigError,
    validate_robot_name,
)
import yaml


FIXED_WING_STOP_ANGULAR_X = 1.0


@dataclass
class ControllerConfig:
    control_mode: str
    pose_source: str
    global_frame: str
    robot_frame: str
    cmd_vel_topic: str
    waypoint_topic: str
    control_rate_hz: float
    goal_tolerance_m: float
    heading_tolerance_deg: float
    forward_linear_x: float
    turn_angular_z: float
    final_loiter_direction: str
    cmd_vel_qos_depth: int
    waypoint_qos_depth: int
    mcs_pose_topic: str
    mcs_frame: str
    mcs_pose_timeout_sec: float
    mcs_qos_depth: int

    @classmethod
    def from_stack_defaults(cls, stack_config, robot_name):
        controller = required(stack_config, 'waypoint_controller')
        mcs = required(stack_config, 'mcs')
        topics = required(stack_config, 'topics')
        return cls(
            control_mode=str(required(stack_config, 'control_mode')),
            pose_source=str(required(stack_config, 'pose_source')),
            global_frame=robot_frame(stack_config, 'map', robot_name),
            robot_frame=robot_frame(stack_config, 'base', robot_name),
            cmd_vel_topic=str(topics['cmd_vel']),
            waypoint_topic=str(topics['waypoints']),
            control_rate_hz=float(controller['control_rate_hz']),
            goal_tolerance_m=float(controller['goal_tolerance_m']),
            heading_tolerance_deg=float(
                controller['heading_tolerance_deg']
            ),
            forward_linear_x=float(controller['forward_linear_x']),
            turn_angular_z=float(controller['turn_angular_z']),
            final_loiter_direction=str(
                controller['final_loiter_direction']
            ),
            cmd_vel_qos_depth=int(controller['cmd_vel_qos_depth']),
            waypoint_qos_depth=int(controller['waypoint_qos_depth']),
            mcs_pose_topic=mcs_pose_topic(stack_config, robot_name),
            mcs_frame=str(mcs['frame']),
            mcs_pose_timeout_sec=float(mcs['pose_timeout_sec']),
            mcs_qos_depth=int(mcs['qos_depth']),
        )


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class PoseUnavailableError(RuntimeError):
    """Raised when the selected pose provider has no current pose."""


class SlamTfPoseProvider:
    """Current SLAM pose provider using the global ROS TF tree."""

    def __init__(self, node, global_frame, robot_frame_id):
        self.global_frame = global_frame
        self.robot_frame = robot_frame_id
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def lookup_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
            )
        except TransformException as error:
            raise PoseUnavailableError(str(error)) from error

        return Pose2D(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            yaw=yaw_from_quaternion(transform.transform.rotation),
        )


class McsPoseProvider:
    """Pose provider using the latest valid external MCS PoseStamped."""

    def __init__(self, node, pose_topic, expected_frame, timeout_sec, depth):
        self.node = node
        self.pose_topic = pose_topic
        self.expected_frame = expected_frame
        self.timeout_sec = timeout_sec
        self.latest_pose = None
        self.latest_pose_received_at = None
        self._last_warning_at = {}
        self._received_first_pose = False
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = node.create_subscription(
            PoseStamped,
            pose_topic,
            self._pose_callback,
            qos,
        )

    def _warn_throttled(self, key, message, period_sec=2.0):
        now = time.monotonic()
        last_warning = self._last_warning_at.get(key)
        if last_warning is not None and now - last_warning < period_sec:
            return
        self._last_warning_at[key] = now
        self.node.get_logger().warn(message)

    def _pose_callback(self, message):
        frame_id = message.header.frame_id.strip()
        if frame_id != self.expected_frame:
            self._warn_throttled(
                'frame',
                'Rejected MCS pose with frame_id="%s"; expected "%s".'
                % (frame_id, self.expected_frame),
            )
            return

        position = message.pose.position
        orientation = message.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self._warn_throttled(
                'non_finite',
                'Rejected MCS pose containing non-finite position or '
                'orientation values.',
            )
            return

        norm = math.hypot(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not math.isfinite(norm) or norm <= 1e-12:
            self._warn_throttled(
                'quaternion',
                'Rejected MCS pose with a zero-length quaternion.',
            )
            return

        inverse_norm = 1.0 / norm
        normalized_orientation = type(orientation)()
        normalized_orientation.x = orientation.x * inverse_norm
        normalized_orientation.y = orientation.y * inverse_norm
        normalized_orientation.z = orientation.z * inverse_norm
        normalized_orientation.w = orientation.w * inverse_norm
        self.latest_pose = Pose2D(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(normalized_orientation),
        )
        self.latest_pose_received_at = time.monotonic()
        if not self._received_first_pose:
            self.node.get_logger().info(
                'Received first valid MCS pose on %s in frame %s.'
                % (self.pose_topic, self.expected_frame)
            )
            self._received_first_pose = True

    def lookup_pose(self):
        if self.latest_pose is None or self.latest_pose_received_at is None:
            raise PoseUnavailableError(
                'no valid MCS pose has been received on %s'
                % self.pose_topic
            )

        age_sec = time.monotonic() - self.latest_pose_received_at
        if age_sec > self.timeout_sec:
            raise PoseUnavailableError(
                'latest MCS pose is stale (age=%.3fs, timeout=%.3fs)'
                % (age_sec, self.timeout_sec)
            )
        return self.latest_pose


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        stack_config = load_stack_config()
        robot_name = validate_robot_name(str(self.declare_parameter(
            'robot_name',
            str(required(stack_config, 'robot_name')),
        ).value))
        params_file = self.declare_parameter('params_file', '').value
        central_controller = required(stack_config, 'waypoint_controller')
        default_control_rate_hz = float(
            central_controller['control_rate_hz']
        )
        default_loiter_direction = str(
            central_controller['final_loiter_direction']
        ).strip().lower()
        config = ControllerConfig.from_stack_defaults(
            stack_config,
            robot_name,
        )
        config = self._load_config(params_file, config)

        try:
            self.control_mode = normalize_control_mode(
                self.declare_parameter(
                    'control_mode',
                    config.control_mode,
                ).value,
                supported=('twist', 'fixed_wing'),
            )
            self.pose_source = normalize_pose_source(
                self.declare_parameter(
                    'pose_source',
                    config.pose_source,
                ).value
            )
        except StackConfigError as error:
            self.get_logger().fatal(str(error))
            raise ValueError(str(error)) from error

        default_global_frame = (
            config.mcs_frame
            if self.pose_source == 'MCS'
            else config.global_frame
        )
        self.global_frame = str(self.declare_parameter(
            'global_frame',
            default_global_frame
        ).value)
        self.robot_frame = str(self.declare_parameter(
            'robot_frame',
            config.robot_frame
        ).value)
        self.cmd_vel_topic = str(self.declare_parameter(
            'cmd_vel_topic',
            config.cmd_vel_topic
        ).value)
        self.waypoint_topic = str(self.declare_parameter(
            'waypoint_topic',
            config.waypoint_topic
        ).value)
        self.control_rate_hz = float(self.declare_parameter(
            'control_rate_hz',
            config.control_rate_hz
        ).value)
        self.goal_tolerance_m = abs(float(self.declare_parameter(
            'goal_tolerance_m',
            config.goal_tolerance_m
        ).value))
        self.heading_tolerance_rad = math.radians(abs(float(
            self.declare_parameter(
                'heading_tolerance_deg',
                config.heading_tolerance_deg
            ).value
        )))
        self.forward_linear_x = max(0.0, float(self.declare_parameter(
            'forward_linear_x',
            config.forward_linear_x
        ).value))
        self.turn_angular_z = abs(float(self.declare_parameter(
            'turn_angular_z',
            config.turn_angular_z
        ).value))
        final_loiter_direction = str(self.declare_parameter(
            'final_loiter_direction',
            config.final_loiter_direction
        ).value).strip().lower()
        if final_loiter_direction not in ('left', 'right'):
            self.get_logger().warn(
                'Invalid final_loiter_direction "%s"; expected "left" or '
                '"right". Falling back to central default "%s".'
                % (final_loiter_direction, default_loiter_direction)
            )
            final_loiter_direction = default_loiter_direction
        self.final_loiter_direction = final_loiter_direction

        if self.control_rate_hz <= 0.0:
            self.get_logger().warn(
                'control_rate_hz must be positive; using central default %.1f '
                'Hz.' % default_control_rate_hz
            )
            self.control_rate_hz = default_control_rate_hz

        self.cmd_vel_qos_depth = max(1, int(self.declare_parameter(
            'cmd_vel_qos_depth',
            config.cmd_vel_qos_depth,
        ).value))
        self.waypoint_qos_depth = max(1, int(self.declare_parameter(
            'waypoint_qos_depth',
            config.waypoint_qos_depth,
        ).value))
        self.mcs_pose_topic = str(self.declare_parameter(
            'mcs_pose_topic',
            config.mcs_pose_topic,
        ).value).strip()
        self.mcs_pose_timeout_sec = float(self.declare_parameter(
            'mcs_pose_timeout_sec',
            config.mcs_pose_timeout_sec,
        ).value)
        self.mcs_qos_depth = int(self.declare_parameter(
            'mcs_qos_depth',
            config.mcs_qos_depth,
        ).value)
        if self.pose_source == 'MCS':
            if not self.mcs_pose_topic.startswith('/'):
                raise ValueError(
                    'mcs_pose_topic must be an absolute topic name.'
                )
            if self.mcs_pose_timeout_sec <= 0.0:
                raise ValueError('mcs_pose_timeout_sec must be positive.')
            if self.mcs_qos_depth <= 0:
                raise ValueError('mcs_qos_depth must be positive.')

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_qos_depth,
        )
        waypoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.waypoint_qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.waypoint_subscription = self.create_subscription(
            PointStamped,
            self.waypoint_topic,
            self._waypoint_callback,
            waypoint_qos,
        )

        if self.pose_source == 'SLAM':
            self.pose_provider = SlamTfPoseProvider(
                self,
                self.global_frame,
                self.robot_frame,
            )
        else:
            self.pose_provider = McsPoseProvider(
                self,
                self.mcs_pose_topic,
                self.global_frame,
                self.mcs_pose_timeout_sec,
                self.mcs_qos_depth,
            )
        self.waypoint_queue = deque()
        self.received_any_waypoint = False
        self.last_bank_direction = None
        self.loiter_direction = None
        self.loitering = False
        self._pose_failure_active = False
        self._last_throttled_log = {}

        self.get_logger().info(
            'control_mode=%s pose_source=%s. Waiting for PointStamped '
            'waypoints on %s.'
            % (
                self.control_mode,
                self.pose_source,
                self.waypoint_subscription.topic_name,
            )
        )
        if self.pose_source == 'SLAM':
            pose_description = 'TF %s -> %s' % (
                self.global_frame,
                self.robot_frame,
            )
        else:
            pose_description = (
                'MCS PoseStamped topic %s in frame %s (timeout %.3f s)'
                % (
                    self.mcs_pose_topic,
                    self.global_frame,
                    self.mcs_pose_timeout_sec,
                )
            )
        self.get_logger().info(
            'Using %s and publishing Twist commands on %s.'
            % (pose_description, self.cmd_vel_publisher.topic_name)
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_step
        )

    def _load_config(self, params_file, config):
        if not params_file:
            return config

        try:
            with open(params_file, 'r', encoding='utf-8') as stream:
                yaml_data = yaml.safe_load(stream) or {}
        except OSError as error:
            self.get_logger().warn(
                f'Could not read params_file "{params_file}": {error}. '
                'Using central stack defaults.'
            )
            return config
        except yaml.YAMLError as error:
            self.get_logger().warn(
                f'Could not parse params_file "{params_file}": {error}. '
                'Using central stack defaults.'
            )
            return config

        params = self._extract_ros_parameters(yaml_data)
        supported_parameters = {
            'control_rate_hz',
            'goal_tolerance_m',
            'heading_tolerance_deg',
            'forward_linear_x',
            'turn_angular_z',
            'final_loiter_direction',
            'cmd_vel_qos_depth',
            'waypoint_qos_depth',
        }
        unknown_parameters = sorted(set(params) - supported_parameters)
        if unknown_parameters:
            raise ValueError(
                'Unknown waypoint parameter overrides: %s.'
                % ', '.join(unknown_parameters)
            )
        config.control_rate_hz = float(
            params.get('control_rate_hz', config.control_rate_hz)
        )
        config.goal_tolerance_m = float(
            params.get('goal_tolerance_m', config.goal_tolerance_m)
        )
        config.heading_tolerance_deg = float(
            params.get('heading_tolerance_deg', config.heading_tolerance_deg)
        )
        config.forward_linear_x = float(
            params.get('forward_linear_x', config.forward_linear_x)
        )
        config.turn_angular_z = float(
            params.get('turn_angular_z', config.turn_angular_z)
        )
        config.final_loiter_direction = str(params.get(
            'final_loiter_direction',
            config.final_loiter_direction
        ))
        config.cmd_vel_qos_depth = int(
            params.get('cmd_vel_qos_depth', config.cmd_vel_qos_depth)
        )
        config.waypoint_qos_depth = int(
            params.get('waypoint_qos_depth', config.waypoint_qos_depth)
        )
        return config

    def _extract_ros_parameters(self, yaml_data):
        if not isinstance(yaml_data, dict):
            self.get_logger().warn(
                'Waypoint config YAML root must be a mapping; using defaults.'
            )
            return {}

        node_params = yaml_data.get('waypoint_controller')
        if isinstance(node_params, dict):
            ros_params = node_params.get('ros__parameters')
            if isinstance(ros_params, dict):
                return ros_params

        wildcard_params = yaml_data.get('/**')
        if isinstance(wildcard_params, dict):
            ros_params = wildcard_params.get('ros__parameters')
            if isinstance(ros_params, dict):
                return ros_params

        ros_params = yaml_data.get('ros__parameters')
        if isinstance(ros_params, dict):
            return ros_params

        return yaml_data

    def _waypoint_callback(self, message):
        frame_id = message.header.frame_id.strip()
        if frame_id != self.global_frame:
            self.get_logger().warn(
                'Rejected waypoint with frame_id="%s"; expected "%s".'
                % (frame_id, self.global_frame)
            )
            return

        x = float(message.point.x)
        y = float(message.point.y)
        z = float(message.point.z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            self.get_logger().warn(
                'Rejected non-finite waypoint (x=%s, y=%s, z=%s).'
                % (x, y, z)
            )
            return

        queue_was_empty = not self.waypoint_queue
        was_loitering = self.loitering
        self.waypoint_queue.append((x, y))
        self.received_any_waypoint = True

        if was_loitering:
            self.loitering = False
            self.loiter_direction = None
            self.get_logger().info(
                'Exiting fixed-wing loiter: a new waypoint was queued.'
            )

        self.get_logger().info(
            'Received waypoint (%.3f, %.3f) in %s; queue_length=%d.'
            % (x, y, frame_id, len(self.waypoint_queue))
        )
        if queue_was_empty:
            self.get_logger().info(
                'Active waypoint is now (%.3f, %.3f).'
                % self.waypoint_queue[0]
            )

    def _control_step(self):
        if not self.waypoint_queue:
            self._control_empty_queue()
            return

        pose = self._lookup_pose_or_stop()
        if pose is None:
            return

        current_x = pose.x
        current_y = pose.y
        current_yaw = pose.yaw

        goal_x, goal_y = self.waypoint_queue[0]
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - current_yaw)

        if distance < self.goal_tolerance_m:
            reached_x, reached_y = self.waypoint_queue.popleft()
            remaining = len(self.waypoint_queue)

            if remaining:
                if self.control_mode == 'fixed_wing':
                    self.publish_forward()
                    command_name = 'straight'
                else:
                    self.publish_stop()
                    command_name = 'stop'
            elif self.control_mode == 'fixed_wing':
                command_name = self._enter_loiter()
            else:
                self.publish_stop()
                command_name = 'stop'

            self.get_logger().info(
                'Reached waypoint (%.3f, %.3f); distance=%.3f m '
                'heading_error=%.3f rad remaining_queue=%d command=%s.'
                % (
                    reached_x,
                    reached_y,
                    distance,
                    heading_error,
                    remaining,
                    command_name,
                )
            )
            if remaining:
                self.get_logger().info(
                    'Active waypoint is now (%.3f, %.3f).'
                    % self.waypoint_queue[0]
                )
            return

        if self.control_mode == 'fixed_wing':
            if heading_error > self.heading_tolerance_rad:
                command_name = 'bank_left'
                self.publish_bank_left()
            elif heading_error < -self.heading_tolerance_rad:
                command_name = 'bank_right'
                self.publish_bank_right()
            else:
                command_name = 'straight'
                self.publish_forward()
        else:
            if heading_error > self.heading_tolerance_rad:
                command_name = 'turn_left'
                self.publish_turn_left()
            elif heading_error < -self.heading_tolerance_rad:
                command_name = 'turn_right'
                self.publish_turn_right()
            else:
                command_name = 'forward'
                self.publish_forward()

        self._info_throttled(
            'control_state',
            (
                'control_mode=%s active=(%.3f, %.3f) queue_length=%d '
                'distance=%.3f m heading_error=%.3f rad command=%s'
            ) % (
                self.control_mode,
                goal_x,
                goal_y,
                len(self.waypoint_queue),
                distance,
                heading_error,
                command_name,
            ),
            period_sec=1.0
        )

    def _control_empty_queue(self):
        if not self.received_any_waypoint:
            command_name = self.publish_safe_stop()
            self._info_throttled(
                'waiting_first_waypoint',
                'control_mode=%s queue_length=0 state=waiting_first_waypoint '
                'command=%s'
                % (self.control_mode, command_name),
                period_sec=2.0,
            )
            return

        if self.control_mode == 'twist':
            self.publish_stop()
            self._info_throttled(
                'waiting_for_more_waypoints',
                'control_mode=twist queue_length=0 state=waiting command=stop',
                period_sec=2.0,
            )
            return

        pose = self._lookup_pose_or_stop()
        if pose is None:
            return

        if not self.loitering:
            command_name = self._enter_loiter()
        else:
            command_name = self.publish_final_loiter()

        self._info_throttled(
            'final_loiter',
            'control_mode=fixed_wing queue_length=0 state=loiter command=%s'
            % command_name,
            period_sec=1.0,
        )

    def _lookup_pose_or_stop(self):
        try:
            pose = self.pose_provider.lookup_pose()
        except PoseUnavailableError as error:
            command_name = self.publish_safe_stop()
            self._pose_failure_active = True
            self._warn_throttled(
                'pose_lookup',
                'control_mode=%s pose_source=%s queue_length=%d command=%s: '
                'pose unavailable in global_frame=%s: %s.'
                % (
                    self.control_mode,
                    self.pose_source,
                    len(self.waypoint_queue),
                    command_name,
                    self.global_frame,
                    error,
                )
            )
            return None

        if self._pose_failure_active:
            self.get_logger().info(
                'pose_source=%s restored in global_frame=%s; resuming '
                'controller state.'
                % (
                    self.pose_source,
                    self.global_frame,
                )
            )
            self._pose_failure_active = False
        return pose

    def _enter_loiter(self):
        self.loiter_direction = (
            self.last_bank_direction or self.final_loiter_direction
        )
        self.loitering = True
        command_name = self.publish_final_loiter()
        self.get_logger().info(
            'Waypoint queue complete; entering fixed-wing loiter '
            'direction=%s command=%s.'
            % (self.loiter_direction, command_name)
        )
        return command_name

    def publish_forward(self):
        message = Twist()
        message.linear.x = self.forward_linear_x
        self.cmd_vel_publisher.publish(message)

    def publish_turn_left(self):
        message = Twist()
        message.angular.z = self.turn_angular_z
        self.cmd_vel_publisher.publish(message)

    def publish_turn_right(self):
        message = Twist()
        message.angular.z = -self.turn_angular_z
        self.cmd_vel_publisher.publish(message)

    def publish_bank_left(self):
        message = Twist()
        message.linear.x = self.forward_linear_x
        message.angular.z = self.turn_angular_z
        self.cmd_vel_publisher.publish(message)
        self.last_bank_direction = 'left'

    def publish_bank_right(self):
        message = Twist()
        message.linear.x = self.forward_linear_x
        message.angular.z = -self.turn_angular_z
        self.cmd_vel_publisher.publish(message)
        self.last_bank_direction = 'right'

    def publish_final_loiter(self):
        if self.loiter_direction == 'right':
            self.publish_bank_right()
        else:
            self.publish_bank_left()
        return 'final_loiter_%s' % self.loiter_direction

    def publish_stop(self):
        self.cmd_vel_publisher.publish(Twist())

    def publish_safe_stop(self):
        message = Twist()
        if self.control_mode == 'fixed_wing':
            message.angular.x = FIXED_WING_STOP_ANGULAR_X
        self.cmd_vel_publisher.publish(message)
        return 'stop'

    def _warn_throttled(self, key, message, period_sec=5.0):
        if self._should_log(key, period_sec):
            self.get_logger().warn(message)

    def _info_throttled(self, key, message, period_sec=1.0):
        if self._should_log(key, period_sec):
            self.get_logger().info(message)

    def _should_log(self, key, period_sec):
        now = time.monotonic()
        last_log_time = self._last_throttled_log.get(key)
        if last_log_time is not None and now - last_log_time < period_sec:
            return False

        self._last_throttled_log[key] = now
        return True


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointController()
    interrupted = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if rclpy.ok() and interrupted:
            node.timer.cancel()
            node.publish_safe_stop()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
