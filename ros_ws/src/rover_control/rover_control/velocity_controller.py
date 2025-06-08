#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import numpy as np
from simple_pid import PID
import time

class VelocityController(Node):
    """
    ROS2 node for closed-loop velocity control of the rover motors.
    Uses PID control to maintain desired wheel velocities.
    """
    
    def __init__(self):
        super().__init__('velocity_controller')
        
        # Wheel parameters
        self.wheel_circumference = 0.217  # meters
        self.ticks_per_revolution = 20  # Assuming 20 ticks per revolution, adjust if different
        
        # Time tracking for velocity calculation
        self.last_time = time.time()
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        # Current encoder values
        self.current_left_ticks = 0
        self.current_right_ticks = 0
        
        # Current velocities
        self.current_left_velocity = 0.0
        self.current_right_velocity = 0.0
        
        # PID controllers
        self.left_pid = PID(Kp=0.01, Ki=0, Kd=0)
        self.right_pid = PID(Kp=0.01, Ki=0, Kd=0)
        
        # Set PID output limits to match motor driver range (-1 to 1)
        self.left_pid.output_limits = (-1.0, 1.0)
        self.right_pid.output_limits = (-1.0, 1.0)
        
        # Create subscribers for encoder counts
        self.left_encoder_sub = self.create_subscription(
            Int32,
            'left_encoder_count',
            self.left_encoder_callback,
            10)
        self.right_encoder_sub = self.create_subscription(
            Int32,
            'right_encoder_count',
            self.right_encoder_callback,
            10)
            
        # Create subscribers for velocity setpoints
        self.left_vel_sub = self.create_subscription(
            Float32,
            'left_motor_velocity',
            self.left_velocity_callback,
            10)
        self.right_vel_sub = self.create_subscription(
            Float32,
            'right_motor_velocity',
            self.right_velocity_callback,
            10)
            
        # Create publishers for motor PWM commands
        self.left_pwm_pub = self.create_publisher(Float32, 'left_motor_pwm', 10)
        self.right_pwm_pub = self.create_publisher(Float32, 'right_motor_pwm', 10)
        
        # Create control loop timer (100 Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info('Velocity controller node initialized')
        
    def calculate_angular_velocity(self, current_ticks, last_ticks, dt):
        """
        Calculate angular velocity in rad/s from encoder ticks.
        
        Args:
            current_ticks: Current encoder count
            last_ticks: Previous encoder count
            dt: Time delta in seconds
            
        Returns:
            float: Angular velocity in rad/s
        """
        # Calculate ticks per second
        ticks_per_second = (current_ticks - last_ticks) / dt
        
        # Convert to revolutions per second
        revs_per_second = ticks_per_second / self.ticks_per_revolution
        
        # Convert to radians per second
        return revs_per_second * 2.0 * np.pi
        
    def left_encoder_callback(self, msg):
        """
        Callback for left encoder updates.
        Updates current encoder value.
        
        Args:
            msg (Int32): Current encoder count
        """
        self.current_left_ticks = msg.data
            
    def right_encoder_callback(self, msg):
        """
        Callback for right encoder updates.
        Updates current encoder value.
        
        Args:
            msg (Int32): Current encoder count
        """
        self.current_right_ticks = msg.data
            
    def left_velocity_callback(self, msg):
        """
        Callback for left motor velocity setpoint.
        
        Args:
            msg (Float32): Desired angular velocity in rad/s
        """
        self.left_pid.setpoint = msg.data
        
    def right_velocity_callback(self, msg):
        """
        Callback for right motor velocity setpoint.
        
        Args:
            msg (Float32): Desired angular velocity in rad/s
        """
        self.right_pid.setpoint = msg.data
        
    def control_loop(self):
        """
        Fixed-rate control loop that:
        1. Calculates current velocities from encoder readings
        2. Updates PID controllers
        3. Publishes motor commands
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0:  # Avoid division by zero
            # Calculate current velocities
            self.current_left_velocity = self.calculate_angular_velocity(
                self.current_left_ticks, self.last_left_ticks, dt)
            self.current_right_velocity = self.calculate_angular_velocity(
                self.current_right_ticks, self.last_right_ticks, dt)
            
            # Update PID controllers and get outputs
            left_pwm = self.left_pid(self.current_left_velocity)
            right_pwm = self.right_pid(self.current_right_velocity)
            
            # Publish PWM commands
            self.left_pwm_pub.publish(Float32(data=left_pwm))
            self.right_pwm_pub.publish(Float32(data=right_pwm))
            
            # Update last values
            self.last_left_ticks = self.current_left_ticks
            self.last_right_ticks = self.current_right_ticks
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 