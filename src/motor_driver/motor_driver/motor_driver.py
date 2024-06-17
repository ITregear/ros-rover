#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # Motor control GPIO pins
        self.motor1_pins = (16, 13)   # IN1, IN2 for Motor 1
        self.motor2_pins = (19, 26)  # IN1, IN2 for Motor 2

        # Differential drive parameters
        self.wheel_base = 0.26  # Distance between wheels in meters (adjust this as per your rover's geometry)
        self.wheel_circumference = 0.217 
        self.spin_slip_factor = 1.76

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor1_pins, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motor2_pins, GPIO.OUT, initial=GPIO.LOW)

        # Subscribe to motor command topics
        self.subscription1 = self.create_subscription(
            Int32,
            'motor1_velocity',
            self.motor1_callback,
            10
        )
        self.subscription2 = self.create_subscription(
            Int32,
            'motor2_velocity',
            self.motor2_callback,
            10
        )

        # Subscribe to twist command topic
        self.subscription_twist = self.create_subscription(
            Twist,
            'cmd_vel',  # Assuming 'cmd_vel' is the topic name used by the teleop node
            self.twist_callback,
            10
        )

        self.get_logger().info("Motor driver node has started.")

    def motor1_callback(self, msg):
        self.set_motor_velocity(self.motor1_pins, msg.data)

    def motor2_callback(self, msg):
        self.set_motor_velocity(self.motor2_pins, msg.data)

    def set_motor_velocity(self, motor_pins, velocity):
        in1_pin, in2_pin = motor_pins
        if velocity > 0:
            # Forward
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
        elif velocity < 0:
            # Reverse
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)
        else:
            # Stop
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.LOW)
        self.get_logger().info(f"Set motor {motor_pins} velocity to {velocity}")

    def twist_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate wheel velocities based on differential drive kinematics
        v_left = linear_velocity - (angular_velocity * self.wheel_base / 2)
        v_right = linear_velocity + (angular_velocity * self.wheel_base / 2)

        # Log the velocities for debugging
        self.get_logger().info(f"Twist received: linear={linear_velocity}, angular={angular_velocity}")
        self.get_logger().info(f"Calculated wheel velocities: left={v_left}, right={v_right}")

        # Send wheel velocities to the motor control functions
        self.set_motor_velocity(self.motor1_pins, v_left)
        self.set_motor_velocity(self.motor2_pins, v_right)

    def close(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
