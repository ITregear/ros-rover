#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

class EncoderPublisher(Node):

    def __init__(self):
        super().__init__('encoder_publisher')
        self.encoder_left_pub = self.create_publisher(Int32, 'encoder_left', 10)
        self.encoder_right_pub = self.create_publisher(Int32, 'encoder_right', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # GPIO setup
        self.encoder_left_pin = 20  # GPIO pin for left encoder
        self.encoder_right_pin = 21  # GPIO pin for right encoder
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoder_left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder_right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize encoder counts
        self.encoder_left_count = 0
        self.encoder_right_count = 0

        # Set up GPIO event detection for pulse counting
        GPIO.add_event_detect(self.encoder_left_pin, GPIO.BOTH, callback=self.left_encoder_callback, bouncetime=10)
        GPIO.add_event_detect(self.encoder_right_pin, GPIO.BOTH, callback=self.right_encoder_callback, bouncetime=10)

        # Add logging
        self.get_logger().info("EncoderPublisher node has started.")
        self.get_logger().info(f"Listening on GPIO pins {self.encoder_left_pin} and {self.encoder_right_pin}.")

    def timer_callback(self):
        left_msg = Int32()
        right_msg = Int32()

        left_msg.data = self.encoder_left_count
        right_msg.data = self.encoder_right_count

        self.encoder_left_pub.publish(left_msg)
        self.encoder_right_pub.publish(right_msg)

    def left_encoder_callback(self, channel):
        self.encoder_left_count += 1

    def right_encoder_callback(self, channel):
        self.encoder_right_count += 1

    def close(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
