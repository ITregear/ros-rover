#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class MotorDriver(Node):
    """
    ROS2 node for controlling DC motors using L298N driver.
    
    This node subscribes to velocity commands and controls the motors using PWM.
    The velocity is currently implemented as an open-loop control using PWM values.
    """
    
    def __init__(self):
        super().__init__('motor_driver')
        
        # Motor control GPIO pins
        self.motor1_pins = (16, 13)  # IN1, IN2 for Motor 1
        self.motor2_pins = (19, 26)  # IN1, IN2 for Motor 2
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor1_pins, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motor2_pins, GPIO.OUT, initial=GPIO.LOW)
        
        # Setup PWM
        self.motor1_pwm1 = GPIO.PWM(self.motor1_pins[0], 200)  # 200 Hz
        self.motor1_pwm2 = GPIO.PWM(self.motor1_pins[1], 200)  # 200 Hz
        self.motor2_pwm1 = GPIO.PWM(self.motor2_pins[0], 200)  # 200 Hz
        self.motor2_pwm2 = GPIO.PWM(self.motor2_pins[1], 200)  # 200 Hz
        self.motor1_pwm1.start(0)
        self.motor1_pwm2.start(0)
        self.motor2_pwm1.start(0)
        self.motor2_pwm2.start(0)
        
        # Create subscribers for left and right motor velocities
        self.left_vel_sub = self.create_subscription(
            Float32,
            'left_motor_pwm',
            self.left_motor_callback,
            10)
        self.right_vel_sub = self.create_subscription(
            Float32,
            'right_motor_pwm',
            self.right_motor_callback,
            10)
            
        self.get_logger().info('Motor driver node initialized')
    
    def set_motor_pwm(self, pwm1, pwm2, velocity):
        """
        Set motor velocity using PWM and direction pins.
        
        Args:
            pwm1: First PWM object for the motor
            pwm2: Second PWM object for the motor
            velocity: Velocity command (-1.0 to 1.0)
        """
        duty_cycle = min(max(abs(velocity) * 100, 0), 100)  # Convert velocity to percentage for PWM
        
        if velocity > 0:
            # Forward
            pwm1.ChangeDutyCycle(duty_cycle)
            pwm2.ChangeDutyCycle(0)
        elif velocity < 0:
            # Reverse
            pwm1.ChangeDutyCycle(0)
            pwm2.ChangeDutyCycle(duty_cycle)
        else:
            # Stop
            pwm1.ChangeDutyCycle(0)
            pwm2.ChangeDutyCycle(0)
    
    def left_motor_callback(self, msg):
        """
        Callback for left motor velocity command.
        
        Args:
            msg (Float32): Velocity command (-1.0 to 1.0)
        """
        self.set_motor_pwm(self.motor1_pwm1, self.motor1_pwm2, msg.data)
    
    def right_motor_callback(self, msg):
        """
        Callback for right motor velocity command.
        
        Args:
            msg (Float32): Velocity command (-1.0 to 1.0)
        """
        self.set_motor_pwm(self.motor2_pwm1, self.motor2_pwm2, msg.data)
    
    def close(self):
        """Cleanup GPIO on node destruction."""
        self.motor1_pwm1.stop()
        self.motor1_pwm2.stop()
        self.motor2_pwm1.stop()
        self.motor2_pwm2.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 