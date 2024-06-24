import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to configuration and launch files
    ldlidar_package_share = get_package_share_directory('ldlidar_node')
    ldlidar_launch_path = os.path.join(ldlidar_package_share, 'launch', 'ldlidar.launch.py')
    lifecycle_mgr_config_path = os.path.join(ldlidar_package_share, 'params', 'lifecycle_mgr.yaml')

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
        # Lifecycle Manager for LiDAR
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_lidar',
            output='screen',
            parameters=[lifecycle_mgr_config_path]
        ),
    ])
