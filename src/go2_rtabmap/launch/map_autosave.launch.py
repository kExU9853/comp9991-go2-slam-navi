from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler
)
from launch.event_handlers import OnShutdown
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments for configuration
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Open RVIZ for visualization'
        ),
        DeclareLaunchArgument(
            name='localize_only',
            default_value='true',
            choices=['true', 'false'],
            description='Localize only, do not change loaded map'
        ),
        DeclareLaunchArgument(
            name='restart_map',
            default_value='true',
            choices=['true', 'false'],
            description='Delete previous map and restart'
        ),
        DeclareLaunchArgument(
            name='map_name',
            default_value='/home/xk/maps/my_map',
            description='Full path to save the map (without .pgm/.yaml)'
        ),

        # Include Hesai driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('hesai_ros_driver'),
                    'launch',
                    'start.py'
                ])
            )
        ),

        # Include RTAB-Map lidar-based mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go2_rtabmap'),
                    'launch',
                    'rtabmaplidar.launch.py'
                ])
            ),
            launch_arguments=[
                ('use_rviz', LaunchConfiguration('use_rviz')),
                ('localize_only', LaunchConfiguration('localize_only')),
                ('restart_map', LaunchConfiguration('restart_map')),
            ],
        ),

        # Register shutdown event to auto-save map
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'run', 'rtabmap_util', 'map_saver',
                            '-f', LaunchConfiguration('map_name')
                        ],
                        output='screen'
                    )
                ]
            )
        )
    ])
