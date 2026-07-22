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
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Absolute PC experiment YAML path',
        ),
        DeclareLaunchArgument(
            'algorithm',
            default_value='',
            description=(
                'Optional config override: heuristic, heuristic_decentralized, '
                'convex, mpc_centralized, or mpc_distributed'
            ),
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='true',
            description='Compute and visualize without publishing commands',
        ),
        Node(
            package='waverover_swarm_controller',
            executable='coordinator',
            name='waverover_swarm_controller',
            namespace='waverover_swarm',
            output='screen',
            parameters=[{
                'config_file': ParameterValue(
                    LaunchConfiguration('config_file'), value_type=str
                ),
                'algorithm': ParameterValue(
                    LaunchConfiguration('algorithm'), value_type=str
                ),
                'dry_run': ParameterValue(
                    LaunchConfiguration('dry_run'), value_type=bool
                ),
            }],
        ),
    ])
