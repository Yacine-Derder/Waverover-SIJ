from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waverover.stack_config import (
    load_stack_config,
    required,
    robot_frame,
    robot_namespace,
    validate_robot_name,
)


STACK_DEFAULTS = load_stack_config()


def _launch_rf2o(context):
    robot_name = validate_robot_name(
        LaunchConfiguration('robot_name').perform(context)
    )
    robot_ns = robot_namespace(STACK_DEFAULTS, robot_name)
    topics = required(STACK_DEFAULTS, 'topics')
    rf2o = required(STACK_DEFAULTS, 'rf2o')

    return [Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name=required(STACK_DEFAULTS, 'nodes', 'rf2o'),
        namespace=robot_ns,
        output='screen',
        parameters=[{
            'laser_scan_topic': topics['scan'],
            'odom_topic': topics['odom'],
            'publish_tf': bool(rf2o['publish_tf']),
            'base_frame_id': robot_frame(
                STACK_DEFAULTS,
                'base',
                robot_name,
            ),
            'odom_frame_id': robot_frame(
                STACK_DEFAULTS,
                'odom',
                robot_name,
            ),
            'init_pose_from_topic': rf2o['init_pose_from_topic'],
            'freq': float(rf2o['frequency_hz']),
        }],
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
        OpaqueFunction(function=_launch_rf2o),
    ])
