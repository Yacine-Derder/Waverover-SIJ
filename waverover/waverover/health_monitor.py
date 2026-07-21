"""Onboard local-heartbeat supervisor; systemd owns stack recovery."""

from dataclasses import dataclass
import json
import os
import time

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .stack_config import load_stack_config, mcs_pose_topic, required


SAFE_NAVIGATION_STATES = {
    'waiting', 'waiting_first_waypoint', 'trial_ended', 'loiter', 'final_loiter'
}


@dataclass
class WatchdogState:
    startup_at: float
    consecutive_failures: int = 0

    def evaluate(self, now, startup_grace, threshold, control_mode,
                 cmd_age, imu_age, imu_enabled, navigation_state=''):
        if now - self.startup_at < startup_grace:
            self.consecutive_failures = 0
            return None
        reasons = []
        if (
            control_mode != 'manual_lr' and
            navigation_state not in SAFE_NAVIGATION_STATES and
            (cmd_age is None or cmd_age > threshold['cmd_vel'])
        ):
            reasons.append('cmd_vel_stale')
        if imu_enabled and (imu_age is None or imu_age > threshold['imu']):
            reasons.append('imu_stale')
        if reasons:
            self.consecutive_failures += 1
        else:
            self.consecutive_failures = 0
        return reasons or None


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        config = load_stack_config()
        watchdog = required(config, 'watchdog')
        topics = required(config, 'topics')
        self.control_mode = str(self.declare_parameter(
            'control_mode', str(required(config, 'control_mode'))
        ).value)
        self.imu_enabled = bool(self.declare_parameter(
            'enable_imu_stream',
            bool(required(config, 'bridge', 'enable_imu_stream')),
        ).value)
        self.startup_grace = float(watchdog['startup_grace_sec'])
        self.threshold = {
            'cmd_vel': float(watchdog['cmd_vel_timeout_sec']),
            'imu': float(watchdog['imu_timeout_sec']),
        }
        self.required_failures = int(watchdog['consecutive_failure_checks'])
        self.last = {'cmd_vel': None, 'imu': None, 'mcs': None}
        self.navigation_state = ''
        self.serial_counters = {}
        self.state = WatchdogState(time.monotonic())
        self.health_publisher = self.create_publisher(String, 'health', 10)
        self.create_subscription(
            Twist, topics['cmd_vel'], lambda _msg: self._seen('cmd_vel'), 10
        )
        self.create_subscription(
            Imu, topics['imu'], lambda _msg: self._seen('imu'), 10
        )
        self.create_subscription(
            String, topics['navigation_status'], self._navigation, 10
        )
        self.create_subscription(
            String, topics['serial_health'], self._serial_health, 10
        )
        robot_name = str(required(config, 'robot_name'))
        self.create_subscription(
            PoseStamped, mcs_pose_topic(config, robot_name),
            lambda _msg: self._seen('mcs'), 10,
        )
        self.waypoint_topic = topics['waypoints']
        self.create_timer(float(watchdog['check_period_sec']), self._check)

    def _seen(self, name):
        self.last[name] = time.monotonic()

    def _navigation(self, message):
        try:
            self.navigation_state = str(json.loads(message.data).get('state', ''))
        except (ValueError, TypeError):
            self.navigation_state = 'malformed_status'

    def _serial_health(self, message):
        try:
            self.serial_counters = json.loads(message.data)
        except (ValueError, TypeError):
            self.serial_counters = {'state': 'malformed'}

    def _age(self, name, now):
        return None if self.last[name] is None else now - self.last[name]

    def _check(self):
        now = time.monotonic()
        ages = {name: self._age(name, now) for name in self.last}
        reasons = self.state.evaluate(
            now, self.startup_grace, self.threshold, self.control_mode,
            ages['cmd_vel'], ages['imu'], self.imu_enabled,
            self.navigation_state,
        )
        restarting = bool(
            reasons and self.state.consecutive_failures >= self.required_failures
        )
        payload = {
            'state': 'restarting' if restarting else (
                'degraded' if reasons else 'healthy'
            ),
            'cmd_vel_age_sec': ages['cmd_vel'],
            'imu_age_sec': ages['imu'],
            'mcs_age_sec': ages['mcs'],  # informational only
            'waypoint_publisher_count': self.count_publishers(
                self.waypoint_topic
            ),  # informational only
            'controller_navigation_state': self.navigation_state,
            'consecutive_failure_checks': self.state.consecutive_failures,
            'restart_reason': ','.join(reasons or ()),
            'serial_counters': self.serial_counters,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        self.health_publisher.publish(message)
        if restarting:
            self.get_logger().fatal(
                'Local heartbeat watchdog exhausted: %s ages=%s' % (
                    payload['restart_reason'], ages
                )
            )
            os._exit(3)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
