from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('waverover_swarm_controller'),
        'config',
        'experiment.yaml',
    ])
    arguments = (
        ('config_file', default_config, str),
        ('rate_hz', '20.0', float),
        # Empty means use synthetic_mcs.initial_radius_m from the YAML.
        ('radius_m', '', str),
        ('angle_offset_rad', '0.0', float),
        ('yaw_rad', '0.0', float),
    )
    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default, _value_type in arguments
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _default, value_type in arguments
    }
    return LaunchDescription(declarations + [
        Node(
            package='waverover_swarm_controller',
            executable='synthetic_mcs',
            name='synthetic_mcs',
            namespace='waverover_swarm',
            output='screen',
            parameters=[parameters],
        ),
    ])
