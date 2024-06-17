#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # Motor control GPIO pins
        self.motor1_pins = (16, 13)   # IN1, IN2 for Motor 1
        self.motor2_pins = (19, 26)  # IN1, IN2 for Motor 2

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor1_pins, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motor2_pins, GPIO.OUT, initial=GPIO.LOW)

        # Subscribe to motor command topic
        self.subscription = self.create_subscription(
            Int32,
            'motor1_velocity',
            self.motor1_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int32,
            'motor2_velocity',
            self.motor2_callback,
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
