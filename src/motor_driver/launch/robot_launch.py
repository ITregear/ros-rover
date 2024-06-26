from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='encoder_publisher',
            executable='encoder_publisher',
            name='encoder_publisher',
            output='screen'
        ),
        Node(
            package='motor_driver',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        ),
    ])
