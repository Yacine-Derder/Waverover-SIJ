import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from waverover.stack_config import load_stack_config, required


STACK_DEFAULTS = load_stack_config(require_identity=False)


def _current_terminal_device():
    try:
        if sys.stdin.isatty():
            return os.ttyname(sys.stdin.fileno())
    except (AttributeError, OSError, ValueError):
        pass
    return ''


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            description='Initial robot ID used by the terminal waypoint UI',
        ),
        DeclareLaunchArgument(
            'pose_source',
            default_value=str(required(STACK_DEFAULTS, 'pose_source')),
            description=(
                'Coordinate source for waypoint frame selection: SLAM or MCS'
            ),
        ),
        DeclareLaunchArgument(
            'refresh_rate_hz',
            default_value=str(required(
                STACK_DEFAULTS,
                'waypoint_ui',
                'refresh_rate_hz',
            )),
            description='Rate for refreshing each rover latest waypoint',
        ),
        DeclareLaunchArgument(
            'terminal_device',
            default_value=_current_terminal_device(),
            description=(
                'Terminal device used for interactive input; detected '
                'automatically for local, VS Code, and SSH terminals'
            ),
        ),
        Node(
            package='waverover_waypoint_ui',
            executable='waypoint_ui',
            name=required(STACK_DEFAULTS, 'nodes', 'waypoint_ui'),
            output='screen',
            parameters=[{
                'robot_name': ParameterValue(
                    LaunchConfiguration('robot_name'),
                    value_type=str,
                ),
                'pose_source': ParameterValue(
                    LaunchConfiguration('pose_source'),
                    value_type=str,
                ),
                'refresh_rate_hz': ParameterValue(
                    LaunchConfiguration('refresh_rate_hz'),
                    value_type=float,
                ),
                'terminal_device': ParameterValue(
                    LaunchConfiguration('terminal_device'),
                    value_type=str,
                ),
            }],
        ),
    ])
