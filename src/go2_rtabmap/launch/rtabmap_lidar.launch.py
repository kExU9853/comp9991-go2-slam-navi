import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def load_config():
    config_path = Path(__file__).parent.parent / 'config' / 'rtabmap_lidar_config.yaml'
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('launch_args', {})

def generate_launch_description():
    cfg = load_config()

    declared_args = [
        DeclareLaunchArgument(
            name=arg,
            default_value=str(cfg[arg]).lower() if isinstance(cfg[arg], bool) else str(cfg[arg]),
            choices=['true', 'false'] if isinstance(cfg[arg], bool) else None,
            description=desc
        )
        for arg, desc in {
            'use_sim_time': 'Use simulation (Gazebo) clock if true',
            'deskewing': 'Enable lidar deskewing',
            'localize_only': 'Localize only, do not add new places to the map',
            'restart_map': 'Delete previous map/database and restart',
            'use_rtabmapviz': 'Start rtabmapviz node',
            'rtabmap_log_level': 'Set logger level for rtabmap.',
            'icp_odometry_log_level': 'Set logger level for icp_odometry.',
        }.items()
    ]

    return LaunchDescription(declared_args + [
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'wait_for_transform': 0.3,
                'expected_update_rate': 15.0,
                'subscribe_odom_info': False,
                'deskewing': LaunchConfiguration('deskewing'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('scan_cloud', '/lidar_points')],
            arguments=[
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/VoxelSize', '0.1',
                'Icp/Epsilon', '0.001',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/MaxTranslation', '2',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.01',
                'Odom/ScanKeyFrameThr', '0.6',
                'OdomF2M/ScanSubtractRadius', '0.1',
                'OdomF2M/ScanMaxSize', '15000',
                'OdomF2M/BundleAdjustment', 'false',
                '--ros-args', '--log-level', LaunchConfiguration('icp_odometry_log_level'),
            ]
        ),

        Node(
            package='rtabmap_util',
            executable='point_cloud_assembler',
            output='screen',
            parameters=[{
                'max_clouds': 10,
                'fixed_frame_id': '',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('cloud', 'odom_filtered_input_scan')]
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'hesai_lidar',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'wait_for_transform': 0.3,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('scan_cloud', 'assembled_cloud')],
            condition=IfCondition(PythonExpression(['"', LaunchConfiguration('restart_map'), '" == "false"'])),
            arguments=[
                'Mem/IncrementalMemory', PythonExpression(['"false" if "', LaunchConfiguration('localize_only'), '" == "true" else "true"']),
                'Mem/InitWMWithAllNodes', PythonExpression(['"true" if "', LaunchConfiguration('localize_only'), '" == "true" else "false"']),
                'Mem/LocalizationDataSaved', PythonExpression(['"false" if "', LaunchConfiguration('localize_only'), '" == "true" else "true"']),
                'RGBD/ProximityMaxGraphDepth', '0',
                'RGBD/ProximityPathMaxNeighbors', '1',
                'RGBD/AngularUpdate', '0.05',
                'RGBD/LinearUpdate', '0.05',
                'RGBD/CreateOccupancyGrid', 'false',
                'Mem/NotLinkedNodesKept', 'false',
                'Mem/STMSize', '30',
                'Mem/LaserScanNormalK', '20',
                'Reg/Strategy', '1',
                'Icp/VoxelSize', '0.1',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/Epsilon', '0.001',
                'Icp/MaxTranslation', '3',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.2',
                '--ros-args', '--log-level', LaunchConfiguration('rtabmap_log_level'),
            ]
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap_reset',
            output='screen',
            parameters=[{
                'frame_id': 'hesai_lidar',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'wait_for_transform': 0.3,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('scan_cloud', 'assembled_cloud')],
            condition=IfCondition(PythonExpression(['"', LaunchConfiguration('restart_map'), '" == "true"'])),
            arguments=[
                'Mem/IncrementalMemory', PythonExpression(['"false" if "', LaunchConfiguration('localize_only'), '" == "true" else "true"']),
                'Mem/InitWMWithAllNodes', PythonExpression(['"true" if "', LaunchConfiguration('localize_only'), '" == "true" else "false"']),
                'Mem/LocalizationDataSaved', PythonExpression(['"false" if "', LaunchConfiguration('localize_only'), '" == "true" else "true"']),
                '--delete_db_on_start',
                'RGBD/ProximityMaxGraphDepth', '0',
                'RGBD/ProximityPathMaxNeighbors', '1',
                'RGBD/AngularUpdate', '0.05',
                'RGBD/LinearUpdate', '0.05',
                'RGBD/CreateOccupancyGrid', 'false',
                'Mem/NotLinkedNodesKept', 'false',
                'Mem/STMSize', '30',
                'Mem/LaserScanNormalK', '20',
                'Reg/Strategy', '1',
                'Icp/VoxelSize', '0.1',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/Epsilon', '0.001',
                'Icp/MaxTranslation', '3',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.2',
                '--ros-args', '--log-level', LaunchConfiguration('rtabmap_log_level'),
            ]
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'hesai_lidar',
                'odom_frame_id': 'odom',
                'subscribe_odom_info': True,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('scan_cloud', 'odom_filtered_input_scan')],
            condition=IfCondition(LaunchConfiguration('use_rtabmapviz'))
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'hesai_lidar']
        )
    ])
