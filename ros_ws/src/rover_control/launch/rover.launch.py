from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_control',
            executable='encoder_reader',
            name='encoder_reader',
            output='screen'
        ),
        Node(
            package='rover_control',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        )
    ]) 