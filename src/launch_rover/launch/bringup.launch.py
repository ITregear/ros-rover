import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to configuration and launch files
    ldlidar_package_share = get_package_share_directory('ldlidar_node')
    ldlidar_launch_path = os.path.join(ldlidar_package_share, 'launch', 'ldlidar.launch.py')
    lifecycle_mgr_slam_config_path = os.path.join(ldlidar_package_share, 'params', 'lifecycle_mgr_slam.yaml')
    slam_toolbox_config_path = os.path.join(ldlidar_package_share, 'params', 'slam_toolbox.yaml')

    return LaunchDescription([
        # Encoder Publisher Node
        Node(
            package='encoder_publisher',
            executable='encoder_publisher',
            name='encoder_publisher',
            output='screen',
            parameters=[{'param_name': 'param_value'}]  # Adjust parameters as needed
        ),
        # Motor Driver Node
        Node(
            package='motor_driver',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            parameters=[{'param_name': 'param_value'}]  # Adjust parameters as needed
        ),
        # Include LDLidar launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch_path),
            launch_arguments={
                'node_name': 'ldlidar_node'
            }.items()
        ),
        # Lifecycle Manager for SLAM
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            namespace='',
            output='screen',
            parameters=[lifecycle_mgr_slam_config_path]
        ),
        # SLAM Toolbox node in async mode
        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=[slam_toolbox_config_path],
            remappings=[
                ('/scan', '/ldlidar_node/scan')
            ]
        ),
        # Real odom publisher
        Node(
            package='odometry_publisher',
            executable='odometry_publisher',
            name='odometry_publisher',
            output='screen',
            parameters=[
                {'wheel_base': 0.26},
                {'wheel_radius': 0.069},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_link'}
            ]
        ),
        # Static transform publisher for map to scan
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_to_scan',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'scan']
        ),
        # Fake odom publisher (if needed, otherwise remove this)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            namespace='',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'ldlidar_base']
        ),
    ])
