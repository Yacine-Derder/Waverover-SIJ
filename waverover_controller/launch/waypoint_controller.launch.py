from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from waverover.stack_config import (
    launch_text,
    load_stack_config,
    mcs_pose_topic,
    normalize_control_mode,
    normalize_pose_source,
    required,
    robot_frame,
    robot_namespace,
    validate_robot_name,
)


STACK_DEFAULTS = load_stack_config()


def _launch_waypoint_controller(context):
    robot_name = validate_robot_name(
        LaunchConfiguration('robot_name').perform(context)
    )
    control_mode = normalize_control_mode(
        LaunchConfiguration('control_mode').perform(context),
        supported=('twist', 'fixed_wing'),
    )
    pose_source = normalize_pose_source(
        LaunchConfiguration('pose_source').perform(context)
    )
    robot_ns = robot_namespace(STACK_DEFAULTS, robot_name)
    topics = required(STACK_DEFAULTS, 'topics')
    mcs = required(STACK_DEFAULTS, 'mcs')
    selected_mcs_topic = LaunchConfiguration(
        'mcs_pose_topic'
    ).perform(context).strip()
    if not selected_mcs_topic:
        selected_mcs_topic = mcs_pose_topic(STACK_DEFAULTS, robot_name)
    selected_mcs_frame = LaunchConfiguration(
        'mcs_frame'
    ).perform(context).strip()
    if not selected_mcs_frame:
        selected_mcs_frame = str(mcs['frame'])
    global_frame = (
        selected_mcs_frame
        if pose_source == 'MCS'
        else robot_frame(STACK_DEFAULTS, 'map', robot_name)
    )
    parameters = [{
        'robot_name': robot_name,
        'params_file': LaunchConfiguration('params_file'),
        'control_mode': control_mode,
        'pose_source': pose_source,
        'global_frame': global_frame,
        'robot_frame': robot_frame(STACK_DEFAULTS, 'base', robot_name),
        'cmd_vel_topic': topics['cmd_vel'],
        'waypoint_topic': topics['waypoints'],
        'mcs_pose_topic': selected_mcs_topic,
        'mcs_pose_timeout_sec': ParameterValue(
            LaunchConfiguration('mcs_pose_timeout_sec'),
            value_type=float,
        ),
        'mcs_qos_depth': int(mcs['qos_depth']),
    }]

    return [Node(
        package='waverover_controller',
        executable='waypoint_controller',
        name=required(STACK_DEFAULTS, 'nodes', 'waypoint_controller'),
        namespace=robot_ns,
        output='screen',
        parameters=parameters,
        remappings=[
            ('tf', topics['tf']),
            ('tf_static', topics['tf_static']),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=str(required(STACK_DEFAULTS, 'robot_name')),
            description='Robot ID used to derive all robot-specific names',
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value=str(required(STACK_DEFAULTS, 'control_mode')),
            description='Waypoint command mode: twist or fixed_wing',
        ),
        DeclareLaunchArgument(
            'pose_source',
            default_value=str(required(STACK_DEFAULTS, 'pose_source')),
            description='Pose source: SLAM (TF) or MCS (PoseStamped)',
        ),
        DeclareLaunchArgument(
            'mcs_pose_topic',
            default_value='',
            description=(
                'Optional absolute MCS PoseStamped topic override; empty '
                'derives it from robot_name and the central topic pattern'
            ),
        ),
        DeclareLaunchArgument(
            'mcs_frame',
            default_value=str(required(STACK_DEFAULTS, 'mcs', 'frame')),
            description='Expected MCS and MCS-waypoint frame ID',
        ),
        DeclareLaunchArgument(
            'mcs_pose_timeout_sec',
            default_value=launch_text(required(
                STACK_DEFAULTS,
                'mcs',
                'pose_timeout_sec',
            )),
            description='Maximum MCS pose age before a safe stop',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description=(
                'Optional waypoint-controller-only ROS parameter overrides; '
                'central defaults are used when empty'
            ),
        ),
        OpaqueFunction(function=_launch_waypoint_controller),
    ])
