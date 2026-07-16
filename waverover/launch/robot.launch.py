import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from waverover.stack_config import (
    launch_text,
    load_stack_config,
    normalize_control_mode,
    normalize_pose_source,
    required,
    validate_robot_name,
)


STACK_DEFAULTS = load_stack_config()


def selected_onboard_components(pose_source, control_mode='fixed_wing'):
    """Return logical components selected by the unified launch."""
    selected_source = normalize_pose_source(pose_source)
    selected_mode = normalize_control_mode(control_mode)
    components = ['bridge']
    if selected_source == 'SLAM':
        components[:0] = ['lidar', 'static_tf', 'rf2o', 'slam']
    if selected_mode != 'manual_lr':
        components.append('waypoint_controller')
    return tuple(components)


def _include(package_name, launch_file, arguments):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_file,
    )
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_path),
        launch_arguments=arguments.items(),
    )


def _launch_onboard_stack(context):
    robot_name = validate_robot_name(
        LaunchConfiguration('robot_name').perform(context)
    )
    control_mode = normalize_control_mode(
        LaunchConfiguration('control_mode').perform(context)
    )
    pose_source = normalize_pose_source(
        LaunchConfiguration('pose_source').perform(context)
    )

    bridge_arguments = {
        'robot_name': robot_name,
        'control_mode': control_mode,
        'UART_address': LaunchConfiguration('UART_address'),
        'baud_rate': LaunchConfiguration('baud_rate'),
        'enable_imu_stream': LaunchConfiguration('enable_imu_stream'),
        'imu_rate_hz': LaunchConfiguration('imu_rate_hz'),
        'manual_lr_timeout_sec': LaunchConfiguration(
            'manual_lr_timeout_sec'
        ),
    }
    controller_arguments = {
        'robot_name': robot_name,
        'control_mode': control_mode,
        'pose_source': pose_source,
        'params_file': LaunchConfiguration('waypoint_params_file'),
        'mcs_pose_topic': LaunchConfiguration('mcs_pose_topic'),
        'mcs_frame': LaunchConfiguration('mcs_frame'),
        'mcs_pose_timeout_sec': LaunchConfiguration(
            'mcs_pose_timeout_sec'
        ),
    }

    actions = []
    if pose_source == 'SLAM':
        slam_arguments = dict(bridge_arguments)
        slam_arguments.update({
            'pose_source': 'SLAM',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'lidar_z': LaunchConfiguration('lidar_z'),
            'lidar_yaw': LaunchConfiguration('lidar_yaw'),
            'laser_scan_dir': LaunchConfiguration('laser_scan_dir'),
            'start_foxglove': LaunchConfiguration('start_foxglove'),
            'foxglove_max_qos_depth': LaunchConfiguration(
                'foxglove_max_qos_depth'
            ),
        })
        actions.append(_include('waverover', 'slam.launch.py', slam_arguments))
    else:
        actions.append(_include(
            'ros2waverover',
            'wave_rover_launch.py',
            bridge_arguments,
        ))

    if control_mode == 'manual_lr':
        actions.append(LogInfo(msg=(
            'control_mode=manual_lr: waypoint controller is not started; '
            'use the manual L/R command source.'
        )))
    else:
        actions.append(_include(
            'waverover_controller',
            'waypoint_controller.launch.py',
            controller_arguments,
        ))

    selected = ', '.join(selected_onboard_components(
        pose_source,
        control_mode,
    ))
    actions.insert(0, LogInfo(msg=(
        'WaveRover onboard stack: robot_name=%s control_mode=%s '
        'pose_source=%s components=[%s]'
        % (robot_name, control_mode, pose_source, selected)
    )))
    if pose_source == 'MCS':
        actions.insert(1, LogInfo(msg=(
            'MCS bridge is external and is not launched onboard.'
        )))
    return actions


def generate_launch_description():
    communication = required(STACK_DEFAULTS, 'communication')
    bridge = required(STACK_DEFAULTS, 'bridge')
    lidar = required(STACK_DEFAULTS, 'lidar')
    slam = required(STACK_DEFAULTS, 'slam')
    mcs = required(STACK_DEFAULTS, 'mcs')
    foxglove = required(STACK_DEFAULTS, 'foxglove')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=str(required(STACK_DEFAULTS, 'robot_name')),
            description='Robot ID used to derive namespaces/topics/frames',
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value=str(required(STACK_DEFAULTS, 'control_mode')),
            description='twist, fixed_wing, or manual_lr',
        ),
        DeclareLaunchArgument(
            'pose_source',
            default_value=str(required(STACK_DEFAULTS, 'pose_source')),
            description='SLAM starts the mapping pipeline; MCS uses poses',
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
            'manual_lr_timeout_sec',
            default_value=launch_text(bridge['manual_lr_timeout_sec']),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=launch_text(slam['use_sim_time']),
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=str(lidar['port']),
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value=str(required(STACK_DEFAULTS, 'frames', 'lidar')),
        ),
        DeclareLaunchArgument(
            'lidar_z',
            default_value=launch_text(lidar['z']),
        ),
        DeclareLaunchArgument(
            'lidar_yaw',
            default_value=launch_text(lidar['yaw']),
        ),
        DeclareLaunchArgument(
            'laser_scan_dir',
            default_value=launch_text(lidar['scan_counterclockwise']),
        ),
        DeclareLaunchArgument(
            'start_foxglove',
            default_value=launch_text(foxglove['start']),
            description='Used only by the SLAM pipeline',
        ),
        DeclareLaunchArgument(
            'foxglove_max_qos_depth',
            default_value=launch_text(foxglove['max_qos_depth']),
        ),
        DeclareLaunchArgument(
            'waypoint_params_file',
            default_value='',
            description='Optional controller tuning-only YAML override',
        ),
        DeclareLaunchArgument(
            'mcs_pose_topic',
            default_value='',
            description=(
                'Optional absolute MCS PoseStamped topic override; empty '
                'derives the topic from robot_name'
            ),
        ),
        DeclareLaunchArgument(
            'mcs_frame',
            default_value=str(mcs['frame']),
        ),
        DeclareLaunchArgument(
            'mcs_pose_timeout_sec',
            default_value=launch_text(mcs['pose_timeout_sec']),
        ),
        OpaqueFunction(function=_launch_onboard_stack),
    ])
