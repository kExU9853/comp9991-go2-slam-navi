from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# launch the camera firstly.
# ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true 
def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='publish_static_tf',
            default_value='true',
            choices=['true', 'false'],
            description='Publish a static transform between base_link and base_laser for standalone use of this launch file.'
        ),

        DeclareLaunchArgument(
            name='use_rtabmapviz',
            default_value='false',  # suppress incessant VTK 9.0 warnings
            choices=['true', 'false'],
            description='Start rtabmapviz node'
        ),

        DeclareLaunchArgument(
            name='localize_only',
            default_value='false',
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
            name='icp_odometry_log_level',
            default_value='WARN',  # reduce output from this node
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
            description='Set logger level for icp_odometry. Can set to WARN to reduce message output from this node.'
        ),

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Open RVIZ for visualization'
        ),

        # Publish a static transform between base_link and base_laser for standalone use
        # of this launch file
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['--frame-id', 'base_link',
                       '--child-frame-id', 'hesai_lidar'],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['--frame-id', 'hesai_lidar',
                       '--child-frame-id', 'camera_link'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),

        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False,
                         'world_frame': 'enu',
                         'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/imu')]),


        # Include hesai_ros_driver start.py
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
                    'rtabmap_camera.launch.py'
                ])
            ),
            launch_arguments=[
                ('use_rtabmapviz', LaunchConfiguration('use_rtabmapviz')),
                ('icp_odometry_log_level', LaunchConfiguration(
                    'icp_odometry_log_level')),
                ('localize_only', LaunchConfiguration('localize_only')),
                ('restart_map', LaunchConfiguration('restart_map')),
            ],
        ),
    ])
