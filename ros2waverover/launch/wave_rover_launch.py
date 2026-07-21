from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from waverover.stack_config import (
    launch_text,
    load_stack_config,
    normalize_control_mode,
    required,
    robot_frame,
    robot_namespace,
    validate_robot_name,
)


STACK_DEFAULTS = load_stack_config(require_identity=False)


def _launch_bridge(context):
    robot_name = validate_robot_name(
        LaunchConfiguration('robot_name').perform(context)
    )
    control_mode = normalize_control_mode(
        LaunchConfiguration('control_mode').perform(context)
    )
    robot_ns = robot_namespace(STACK_DEFAULTS, robot_name)
    topics = required(STACK_DEFAULTS, 'topics')

    bridge_node = Node(
        package='ros2waverover',
        namespace=robot_ns,
        executable='ros2waverover-node',
        name=required(STACK_DEFAULTS, 'nodes', 'bridge'),
        output='screen',
        parameters=[{
            'UART_address': LaunchConfiguration('UART_address'),
            'baud_rate': ParameterValue(
                LaunchConfiguration('baud_rate'),
                value_type=int,
            ),
            'enable_imu_stream': ParameterValue(
                LaunchConfiguration('enable_imu_stream'),
                value_type=bool,
            ),
            'imu_rate_hz': ParameterValue(
                LaunchConfiguration('imu_rate_hz'),
                value_type=int,
            ),
            'control_mode': control_mode,
            'cmd_vel_topic': topics['cmd_vel'],
            'imu_topic': topics['imu'],
            'serial_health_topic': topics['serial_health'],
            'imu_frame_id': robot_frame(
                STACK_DEFAULTS,
                'base',
                robot_name,
            ),
            'manual_lr_topic': topics['manual_lr'],
            'manual_lr_timeout_sec': ParameterValue(
                LaunchConfiguration('manual_lr_timeout_sec'),
                value_type=float,
            ),
        }],
    )
    return [
        bridge_node,
        RegisterEventHandler(OnProcessExit(
            target_action=bridge_node,
            on_exit=[EmitEvent(event=Shutdown(
                reason='critical serial bridge exited'
            ))],
        )),
    ]


def generate_launch_description():
    onboard_config = load_stack_config(require_identity=True)
    communication = required(STACK_DEFAULTS, 'communication')
    bridge = required(STACK_DEFAULTS, 'bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=str(required(onboard_config, 'robot_name')),
            description='Robot ID used to derive all robot-specific names',
        ),
        DeclareLaunchArgument(
            'UART_address',
            default_value=str(communication['uart_address']),
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value=launch_text(communication['baud_rate']),
        ),
        DeclareLaunchArgument(
            'enable_imu_stream',
            default_value=launch_text(bridge['enable_imu_stream']),
        ),
        DeclareLaunchArgument(
            'imu_rate_hz',
            default_value=launch_text(bridge['imu_rate_hz']),
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value=str(required(STACK_DEFAULTS, 'control_mode')),
            description='twist, fixed_wing, or manual_lr',
        ),
        DeclareLaunchArgument(
            'manual_lr_timeout_sec',
            default_value=launch_text(bridge['manual_lr_timeout_sec']),
            description='Manual wheel-command safety timeout',
        ),
        OpaqueFunction(function=_launch_bridge),
    ])
