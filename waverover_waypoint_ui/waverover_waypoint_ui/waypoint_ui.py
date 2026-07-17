from collections import deque
import math
import select
import signal
import sys

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty

from waverover.stack_config import (
    load_stack_config,
    normalize_pose_source,
    required,
    robot_topic,
    StackConfigError,
    validate_robot_name,
    waypoint_global_frame,
)


STACK_DEFAULTS = load_stack_config(require_identity=False)


def validate_robot_id(value):
    try:
        return validate_robot_name(value)
    except StackConfigError as error:
        raise ValueError(str(error)) from error


def waypoint_topic(robot_id):
    return robot_topic(
        STACK_DEFAULTS,
        'waypoints',
        validate_robot_id(robot_id),
    )


def waypoint_frame(robot_id, pose_source=None):
    return waypoint_global_frame(
        STACK_DEFAULTS,
        pose_source,
        validate_robot_id(robot_id),
    )


def end_trial_topic(robot_id):
    return robot_topic(
        STACK_DEFAULTS,
        'end_trial',
        validate_robot_id(robot_id),
    )


def map_frame(robot_id):
    """Backward-compatible helper for the SLAM waypoint frame."""
    return waypoint_frame(robot_id, 'SLAM')


def parse_terminal_command(command, current_robot_id):
    """
    Parse one terminal command into an action tuple.

    Supported waypoint forms are ``X Y`` for the current robot and
    ``ROBOT_ID X Y`` for an explicitly selected robot.
    """
    current_robot_id = validate_robot_id(current_robot_id)
    tokens = command.replace(',', ' ').split()
    if not tokens:
        return ('empty',)

    keyword = tokens[0].lower()
    if keyword in ('quit', 'exit', 'q'):
        if len(tokens) != 1:
            raise ValueError('quit does not take any arguments.')
        return ('quit',)
    if keyword in ('help', 'h', '?'):
        if len(tokens) != 1:
            raise ValueError('help does not take any arguments.')
        return ('help',)
    if keyword in ('status', 's'):
        if len(tokens) != 1:
            raise ValueError('status does not take any arguments.')
        return ('status',)
    if keyword == 'end':
        if len(tokens) == 1 or (
            len(tokens) == 2 and tokens[1].lower() == 'trial'
        ):
            return ('end',)
        raise ValueError('Use: end or end trial')
    if keyword in ('robot', 'use'):
        if len(tokens) != 2:
            raise ValueError('Use: robot ROBOT_ID')
        return ('robot', validate_robot_id(tokens[1]))

    if len(tokens) == 1:
        return ('robot', validate_robot_id(tokens[0]))
    if len(tokens) == 2:
        robot_id = current_robot_id
        coordinate_tokens = tokens
    elif len(tokens) == 3:
        robot_id = validate_robot_id(tokens[0])
        coordinate_tokens = tokens[1:]
    else:
        raise ValueError(
            'Enter X Y, ROBOT_ID X Y, robot ROBOT_ID, help, or quit.'
        )

    try:
        x = float(coordinate_tokens[0])
        y = float(coordinate_tokens[1])
    except ValueError as error:
        raise ValueError('X and Y must be numbers.') from error
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError('X and Y must be finite numbers.')
    return ('send', robot_id, x, y)


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waverover_waypoint_ui')
        self.default_robot_id = str(self.declare_parameter(
            'robot_name',
            '',
        ).value)
        self.default_robot_id = validate_robot_id(self.default_robot_id)
        self.pose_source = normalize_pose_source(self.declare_parameter(
            'pose_source',
            str(required(STACK_DEFAULTS, 'pose_source')),
        ).value)
        self.terminal_device = str(self.declare_parameter(
            'terminal_device',
            '',
        ).value).strip()
        default_refresh_rate_hz = float(required(
            STACK_DEFAULTS,
            'waypoint_ui',
            'refresh_rate_hz',
        ))
        self.refresh_rate_hz = float(self.declare_parameter(
            'refresh_rate_hz',
            default_refresh_rate_hz,
        ).value)
        if not math.isfinite(self.refresh_rate_hz) or (
            self.refresh_rate_hz <= 0.0
        ):
            self.get_logger().warn(
                'refresh_rate_hz must be positive and finite; using central '
                'default %.1f Hz.' % default_refresh_rate_hz
            )
            self.refresh_rate_hz = default_refresh_rate_hz
        self._waypoint_publishers = {}
        self._end_trial_publishers = {}
        self.latest_waypoints = {}
        self.commanded_robot_ids = set()
        self._cleanup_complete = False
        self._waypoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(required(
                STACK_DEFAULTS,
                'waypoint_ui',
                'publisher_qos_depth',
            )),
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._end_trial_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._refresh_timer = self.create_timer(
            1.0 / self.refresh_rate_hz,
            self.refresh_waypoints,
        )

    def _ensure_publishers(self, robot_id):
        waypoint_publisher = self._waypoint_publishers.get(robot_id)
        if waypoint_publisher is None:
            waypoint_publisher = self.create_publisher(
                PointStamped,
                waypoint_topic(robot_id),
                self._waypoint_qos,
            )
            self._waypoint_publishers[robot_id] = waypoint_publisher

        if robot_id not in self._end_trial_publishers:
            self._end_trial_publishers[robot_id] = self.create_publisher(
                Empty,
                end_trial_topic(robot_id),
                self._end_trial_qos,
            )
        return waypoint_publisher

    def _publish_target(self, robot_id, x, y):
        publisher = self._ensure_publishers(robot_id)
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = waypoint_frame(
            robot_id,
            self.pose_source,
        )
        message.point.x = x
        message.point.y = y
        message.point.z = 0.0
        publisher.publish(message)
        return message

    def publish_waypoint(self, robot_id, x, y):
        robot_id = validate_robot_id(robot_id)
        x = float(x)
        y = float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('X and Y must be finite numbers.')

        message = self._publish_target(robot_id, x, y)
        self.commanded_robot_ids.add(robot_id)
        self.latest_waypoints[robot_id] = (x, y)
        if self._refresh_timer.is_canceled():
            self._refresh_timer.reset()
        return waypoint_topic(robot_id), message.header.frame_id

    def refresh_waypoints(self):
        for robot_id, (x, y) in tuple(self.latest_waypoints.items()):
            self._publish_target(robot_id, x, y)

    def stop_refreshes(self):
        if not self._refresh_timer.is_canceled():
            self._refresh_timer.cancel()
        self.latest_waypoints.clear()

    def publish_end_trial(self):
        robot_ids = sorted(self.commanded_robot_ids)
        for robot_id in robot_ids:
            publisher = self._end_trial_publishers.get(robot_id)
            if publisher is None:
                publisher = self.create_publisher(
                    Empty,
                    end_trial_topic(robot_id),
                    self._end_trial_qos,
                )
                self._end_trial_publishers[robot_id] = publisher
            publisher.publish(Empty())
        return robot_ids

    def end_trial(self):
        self.stop_refreshes()
        return self.publish_end_trial()

    def cleanup(self):
        if self._cleanup_complete:
            return []
        robot_ids = self.end_trial()
        self._cleanup_complete = True
        return robot_ids


class WaypointTerminal:
    def __init__(self, node, input_stream=None, output_stream=None):
        self.node = node
        self.robot_id = validate_robot_id(node.default_robot_id)
        self.recent_sends = deque(maxlen=int(required(
            STACK_DEFAULTS,
            'waypoint_ui',
            'recent_send_limit',
        )))
        self._terminal_streams = []
        terminal_input = None
        terminal_output = None

        if input_stream is None and output_stream is None:
            terminal_candidates = [node.terminal_device, '/dev/tty']
            for terminal_device in terminal_candidates:
                if not terminal_device:
                    continue
                try:
                    # ros2 launch does not forward stdin to child processes.
                    # Its launch file passes the parent's terminal device so
                    # local, VS Code, and SSH shells remain interactive.
                    terminal_input = open(
                        terminal_device,
                        mode='r',
                        encoding='utf-8',
                        buffering=1,
                    )
                    terminal_output = open(
                        terminal_device,
                        mode='w',
                        encoding='utf-8',
                        buffering=1,
                    )
                    self._terminal_streams = [
                        terminal_input,
                        terminal_output,
                    ]
                    break
                except OSError:
                    if terminal_input is not None:
                        terminal_input.close()
                    terminal_input = None
                    terminal_output = None
                    continue

        self.input_stream = (
            input_stream or terminal_input or sys.stdin
        )
        self.output_stream = (
            output_stream or terminal_output or sys.stdout
        )

    def _write(self, text='', end='\n'):
        print(text, end=end, file=self.output_stream, flush=True)

    def _show_help(self):
        self._write('')
        self._write('WaveRover terminal waypoint sender')
        self._write('  X Y              send to the current robot')
        self._write('  ROBOT_ID X Y     select a robot and send')
        self._write('  robot ROBOT_ID   change the current robot')
        self._write('  status           show destination and recent sends')
        self._write('  end / end trial  stop refreshing and end this trial')
        self._write('  help             show this help')
        self._write('  quit             exit')
        self._write('Commas are also accepted, for example: 29, 1.0, -0.5')

    def _show_status(self):
        self._write(
            'Current robot %s -> %s (pose_source=%s, frame %s)'
            % (
                self.robot_id,
                waypoint_topic(self.robot_id),
                self.node.pose_source,
                waypoint_frame(
                    self.robot_id,
                    self.node.pose_source,
                ),
            )
        )
        commanded = sorted(self.node.commanded_robot_ids)
        self._write(
            'Commanded rovers: %s'
            % (', '.join(commanded) if commanded else 'none')
        )
        if self.node.latest_waypoints:
            self._write(
                'Latest refreshed targets (%.3f Hz):'
                % self.node.refresh_rate_hz
            )
            for robot_id, (x, y) in sorted(
                self.node.latest_waypoints.items()
            ):
                self._write(
                    '  robot=%s x=%.3f y=%.3f frame=%s'
                    % (
                        robot_id,
                        x,
                        y,
                        waypoint_frame(robot_id, self.node.pose_source),
                    )
                )
        else:
            self._write('Latest refreshed targets: none')
        if not self.recent_sends:
            self._write('No waypoints sent in this session.')
            return
        self._write('Recent sends:')
        for entry in self.recent_sends:
            self._write('  ' + entry)

    def _read_line(self):
        try:
            file_descriptor = self.input_stream.fileno()
        except (AttributeError, OSError):
            return self.input_stream.readline()

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            try:
                readable, _, _ = select.select(
                    [file_descriptor],
                    [],
                    [],
                    0.1,
                )
            except (OSError, ValueError):
                return self.input_stream.readline()
            if readable:
                return self.input_stream.readline()
        return ''

    def _handle_command(self, command):
        try:
            action = parse_terminal_command(command, self.robot_id)
        except (TypeError, ValueError) as error:
            self._write('Error: %s' % error)
            return True

        if action[0] == 'empty':
            return True
        if action[0] == 'quit':
            return False
        if action[0] == 'help':
            self._show_help()
            return True
        if action[0] == 'status':
            self._show_status()
            return True
        if action[0] == 'end':
            robot_ids = self.node.end_trial()
            self._write(
                'Published end-trial to rover IDs: %s'
                % (', '.join(robot_ids) if robot_ids else 'none')
            )
            return True
        if action[0] == 'robot':
            self.robot_id = action[1]
            self._write(
                'Selected robot %s -> %s'
                % (self.robot_id, waypoint_topic(self.robot_id))
            )
            return True

        _, robot_id, x, y = action
        try:
            topic, frame = self.node.publish_waypoint(robot_id, x, y)
        except (TypeError, ValueError) as error:
            self._write('Error: %s' % error)
            return True

        self.robot_id = robot_id
        entry = 'robot=%s x=%.3f y=%.3f frame=%s' % (
            robot_id,
            x,
            y,
            frame,
        )
        self.recent_sends.appendleft(entry)
        self._write('Published %s to %s' % (entry, topic))
        return True

    def run(self):
        self._show_help()
        self._show_status()
        while rclpy.ok():
            self._write('waypoint[%s]> ' % self.robot_id, end='')
            try:
                command = self._read_line()
            except KeyboardInterrupt:
                self._write('')
                break
            if command == '':
                self._write('Terminal input closed; exiting.')
                break
            if not self._handle_command(command):
                break

    def close(self):
        for terminal_stream in self._terminal_streams:
            terminal_stream.close()
        self._terminal_streams = []


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointPublisher()
    terminal = WaypointTerminal(node)
    previous_handlers = {}

    def _interrupt(_signum, _frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, _interrupt)
    try:
        terminal.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_ids = node.cleanup()
        if robot_ids and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
        terminal.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
