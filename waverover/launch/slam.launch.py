import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition

from waverover.stack_config import (
    launch_text,
    load_stack_config,
    normalize_control_mode,
    normalize_pose_source,
    required,
    robot_frame,
    robot_namespace,
    validate_robot_name,
)


STACK_DEFAULTS = load_stack_config()


def _launch_robot(context):
    robot_name = validate_robot_name(
        LaunchConfiguration('robot_name').perform(context)
    )
    control_mode = normalize_control_mode(
        LaunchConfiguration('control_mode').perform(context)
    )
    pose_source = normalize_pose_source(
        LaunchConfiguration('pose_source').perform(context)
    )
    if pose_source == 'MCS':
        raise RuntimeError(
            'slam.launch.py only starts the SLAM pose pipeline. Use '
            'robot.launch.py for pose_source=MCS.'
        )

    robot_ns = robot_namespace(STACK_DEFAULTS, robot_name)
    map_frame = robot_frame(STACK_DEFAULTS, 'map', robot_name)
    odom_frame = robot_frame(STACK_DEFAULTS, 'odom', robot_name)
    base_frame = robot_frame(STACK_DEFAULTS, 'base', robot_name)
    lidar_frame_name = validate_robot_name(
        LaunchConfiguration('lidar_frame').perform(context)
    )
    lidar_frame = '%s/%s' % (robot_ns, lidar_frame_name)

    topics = required(STACK_DEFAULTS, 'topics')
    nodes = required(STACK_DEFAULTS, 'nodes')
    lidar = required(STACK_DEFAULTS, 'lidar')
    rf2o = required(STACK_DEFAULTS, 'rf2o')
    slam = required(STACK_DEFAULTS, 'slam')

    tf_remappings = [
        ('tf', topics['tf']),
        ('tf_static', topics['tf_static']),
    ]
    slam_remappings = tf_remappings + [
        ('/map', topics['map']),
        ('/map_metadata', topics['map_metadata']),
    ]

    foxglove_share = get_package_share_directory('foxglove_bridge')
    ros2waverover_share = get_package_share_directory('ros2waverover')

    foxglove_launch = os.path.join(
        foxglove_share,
        'launch',
        'foxglove_bridge_launch.xml',
    )
    ros2waverover_launch = os.path.join(
        ros2waverover_share,
        'launch',
        'wave_rover_launch.py',
    )

    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name=nodes['lidar'],
        namespace=robot_ns,
        output='screen',
        parameters=[{
            'product_name': lidar['product_name'],
            'topic_name': topics['scan'],
            'frame_id': lidar_frame,
            'port_name': LaunchConfiguration('lidar_port'),
            'port_baudrate': int(lidar['baud_rate']),
            'laser_scan_dir': ParameterValue(
                LaunchConfiguration('laser_scan_dir'),
                value_type=bool,
            ),
            'enable_angle_crop_func': bool(lidar['enable_angle_crop']),
            'angle_crop_min': float(lidar['angle_crop_min']),
            'angle_crop_max': float(lidar['angle_crop_max']),
            'scan_points': int(lidar['scan_points']),
        }],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_%s' % lidar_frame_name,
        namespace=robot_ns,
        arguments=[
            '0',
            '0',
            LaunchConfiguration('lidar_z'),
            LaunchConfiguration('lidar_yaw'),
            '0',
            '0',
            base_frame,
            lidar_frame,
        ],
        remappings=tf_remappings,
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name=nodes['rf2o'],
        namespace=robot_ns,
        output='screen',
        arguments=['--ros-args', '--log-level', 'error'],
        parameters=[{
            'laser_scan_topic': topics['scan'],
            'odom_topic': topics['odom'],
            'publish_tf': bool(rf2o['publish_tf']),
            'base_frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'init_pose_from_topic': rf2o['init_pose_from_topic'],
            'freq': float(rf2o['frequency_hz']),
        }],
        remappings=tf_remappings,
    )

    bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(ros2waverover_launch),
        launch_arguments={
            'robot_name': robot_name,
            'UART_address': LaunchConfiguration('UART_address'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'enable_imu_stream': LaunchConfiguration('enable_imu_stream'),
            'imu_rate_hz': LaunchConfiguration('imu_rate_hz'),
            'control_mode': control_mode,
            'manual_lr_timeout_sec': LaunchConfiguration(
                'manual_lr_timeout_sec'
            ),
        }.items(),
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name=nodes['slam'],
        namespace=robot_ns,
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'scan_topic': topics['scan'],
            'resolution': float(slam['resolution']),
            'max_laser_range': float(slam['max_laser_range']),
            'minimum_travel_distance': float(
                slam['minimum_travel_distance']
            ),
            'minimum_travel_heading': float(
                slam['minimum_travel_heading']
            ),
            'transform_timeout': float(slam['transform_timeout']),
            'tf_buffer_duration': float(slam['tf_buffer_duration']),
            'transform_publish_period': float(
                slam['transform_publish_period']
            ),
        }],
        remappings=slam_remappings,
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='Activating namespaced SLAM for ' + robot_ns),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch),
        condition=IfCondition(LaunchConfiguration('start_foxglove')),
        launch_arguments={
            'max_qos_depth': LaunchConfiguration('foxglove_max_qos_depth'),
        }.items(),
    )

    return [
        lidar_node,
        static_tf_node,
        rf2o_node,
        bridge,
        slam_node,
        configure_slam,
        activate_slam,
        foxglove,
    ]


def generate_launch_description():
    communication = required(STACK_DEFAULTS, 'communication')
    bridge = required(STACK_DEFAULTS, 'bridge')
    lidar = required(STACK_DEFAULTS, 'lidar')
    foxglove = required(STACK_DEFAULTS, 'foxglove')
    slam = required(STACK_DEFAULTS, 'slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=str(required(STACK_DEFAULTS, 'robot_name')),
            description='Robot ID used to derive all robot-specific names',
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value=str(required(STACK_DEFAULTS, 'control_mode')),
            description='twist, fixed_wing, or manual_lr',
        ),
        DeclareLaunchArgument(
            'pose_source',
            default_value=str(required(STACK_DEFAULTS, 'pose_source')),
            description=(
                'This SLAM-only launch requires SLAM; use robot.launch.py '
                'to select SLAM or MCS'
            ),
        ),
        DeclareLaunchArgument(
            'UART_address',
            default_value=str(communication['uart_address']),
            description='UART device for the WaveRover controller',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value=launch_text(communication['baud_rate']),
            description='UART baud rate',
        ),
        DeclareLaunchArgument(
            'enable_imu_stream',
            default_value=launch_text(bridge['enable_imu_stream']),
            description='Enable raw IMU feedback from the rover bridge',
        ),
        DeclareLaunchArgument(
            'imu_rate_hz',
            default_value=launch_text(bridge['imu_rate_hz']),
            description='WaveRover feedback IMU rate',
        ),
        DeclareLaunchArgument(
            'manual_lr_timeout_sec',
            default_value=launch_text(bridge['manual_lr_timeout_sec']),
            description='Manual wheel-command safety timeout',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=launch_text(slam['use_sim_time']),
            description='Use simulation/Gazebo clock',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=str(lidar['port']),
            description='Serial device for the STL-19P/LD19 LiDAR',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value=str(required(STACK_DEFAULTS, 'frames', 'lidar')),
            description='LiDAR frame basename; robot namespace is prefixed',
        ),
        DeclareLaunchArgument(
            'lidar_z',
            default_value=launch_text(lidar['z']),
            description='LiDAR height above base frame in meters',
        ),
        DeclareLaunchArgument(
            'lidar_yaw',
            default_value=launch_text(lidar['yaw']),
            description='LiDAR yaw relative to base frame in radians',
        ),
        DeclareLaunchArgument(
            'laser_scan_dir',
            default_value=launch_text(lidar['scan_counterclockwise']),
            description='Publish LiDAR angles counterclockwise',
        ),
        DeclareLaunchArgument(
            'start_foxglove',
            default_value=launch_text(foxglove['start']),
            description='Start the one global Foxglove bridge',
        ),
        DeclareLaunchArgument(
            'foxglove_max_qos_depth',
            default_value=launch_text(foxglove['max_qos_depth']),
            description='Maximum Foxglove subscription history depth',
        ),
        OpaqueFunction(function=_launch_robot),
    ])
