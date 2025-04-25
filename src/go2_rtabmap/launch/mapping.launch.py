import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def load_launch_config():
    config_file_path = Path(__file__).parent / '../config/mapping.yaml'
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config.get('launch_args', {})


def generate_launch_description():
    launch_args = load_launch_config()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_rviz',
            default_value=str(launch_args.get('use_rviz', 'true')).lower(),
            choices=['true', 'false'],
            description='Open RVIZ for visualization'
        ),
        DeclareLaunchArgument(
            name='localize_only',
            default_value=str(launch_args.get('localize_only', 'true')).lower(),
            choices=['true', 'false'],
            description='Localize only, do not change loaded map'
        ),
        DeclareLaunchArgument(
            name='restart_map',
            default_value=str(launch_args.get('restart_map', 'true')).lower(),
            choices=['true', 'false'],
            description='Delete previous map and restart'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('hesai_ros_driver'),
                    'launch',
                    'start.py'
                ])
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go2_rtabmap'),
                    'launch',
                    'rtabmap_lidar.launch.py'
                ])
            ),
            launch_arguments=[
                ('use_rviz', LaunchConfiguration('use_rviz')),
                ('localize_only', LaunchConfiguration('localize_only')),
                ('restart_map', LaunchConfiguration('restart_map')),
            ],
        ),
    ])
