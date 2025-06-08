#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

class EncoderReader(Node):
    """
    ROS2 node for reading encoder values using polling-based edge detection.
    """

    def __init__(self):
        super().__init__('encoder_reader')

        # Encoder GPIO pins (BCM numbering)
        self.encoder_left_pin = 20
        self.encoder_right_pin = 21

        # Encoder counts
        self.encoder_left_count = 0
        self.encoder_right_count = 0

        # Previous pin states
        self.prev_left_state = None
        self.prev_right_state = None

        # Publishers
        self.left_encoder_pub = self.create_publisher(Int32, 'left_encoder_count', 10)
        self.right_encoder_pub = self.create_publisher(Int32, 'right_encoder_count', 10)

        # Safe GPIO setup
        self._setup_gpio()

        # Polling timer (1 kHz)
        self.timer = self.create_timer(0.001, self.timer_callback)

        self.get_logger().info("EncoderReader node started and polling at 1kHz.")

    def _setup_gpio(self):
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)

            GPIO.setup(self.encoder_left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.encoder_right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            self.prev_left_state = GPIO.input(self.encoder_left_pin)
            self.prev_right_state = GPIO.input(self.encoder_right_pin)

            self.get_logger().info(f"GPIO setup complete. Left pin={self.encoder_left_pin}, Right pin={self.encoder_right_pin}")
            self.get_logger().info(f"Initial pin states: Left={self.prev_left_state}, Right={self.prev_right_state}")
        except Exception as e:
            self.get_logger().error(f"GPIO setup failed: {e}")
            raise

    def timer_callback(self):
        """Poll the encoder GPIO pins and publish edge count."""
        try:
            current_left = GPIO.input(self.encoder_left_pin)
            current_right = GPIO.input(self.encoder_right_pin)

            if current_left != self.prev_left_state:
                self.encoder_left_count += 1
                self.prev_left_state = current_left

            if current_right != self.prev_right_state:
                self.encoder_right_count += 1
                self.prev_right_state = current_right

            # Publish messages
            self.left_encoder_pub.publish(Int32(data=self.encoder_left_count))
            self.right_encoder_pub.publish(Int32(data=self.encoder_right_count))
        except Exception as e:
            self.get_logger().error(f"Timer callback error: {e}")

    def close(self):
        """Cleanup GPIO safely on shutdown."""
        try:
            GPIO.cleanup()
            self.get_logger().info("GPIO cleanup completed.")
        except Exception as e:
            self.get_logger().error(f"GPIO cleanup error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EncoderReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node interrupted by user (Ctrl+C).")
    except Exception as e:
        print(f"Unhandled exception in main: {e}")
    finally:
        if node is not None:
            node.close()
        rclpy.shutdown()
        time.sleep(0.1)  # Small delay to avoid race conditions during shutdown

if __name__ == '__main__':
    main()
