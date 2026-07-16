from collections import deque
import math
import select
import sys

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

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
        self._waypoint_publishers = {}
        self._waypoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(required(
                STACK_DEFAULTS,
                'waypoint_ui',
                'publisher_qos_depth',
            )),
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def publish_waypoint(self, robot_id, x, y):
        robot_id = validate_robot_id(robot_id)
        x = float(x)
        y = float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('X and Y must be finite numbers.')

        topic = waypoint_topic(robot_id)
        publisher = self._waypoint_publishers.get(robot_id)
        if publisher is None:
            publisher = self.create_publisher(
                PointStamped,
                topic,
                self._waypoint_qos,
            )
            self._waypoint_publishers[robot_id] = publisher

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
        return topic, message.header.frame_id


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
    rclpy.init(args=args)
    node = WaypointPublisher()
    terminal = WaypointTerminal(node)
    try:
        terminal.run()
    except KeyboardInterrupt:
        pass
    finally:
        terminal.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
