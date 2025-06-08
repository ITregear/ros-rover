from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_control',
            executable='encoder_publisher',
            name='encoder_publisher',
            output='screen'
        ),
        Node(
            package='rover_control',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        ),
        Node(
            package='rover_control',
            executable='velocity_controller',
            name='velocity_controller',
            output='screen'
        )
    ]) 