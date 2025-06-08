#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

class EncoderReader(Node):
    """
    ROS2 node for reading encoder values from DC motors.
    
    This node reads the encoder pulses from both motors and publishes
    the count as messages. The encoders are connected to GPIO pins and
    use polling-based counting for accurate readings.
    
    Attributes:
        encoder_left_pin (int): GPIO pin for left encoder (BCM 35)
        encoder_right_pin (int): GPIO pin for right encoder (BCM 37)
        encoder_left_count (int): Current count of left encoder pulses
        encoder_right_count (int): Current count of right encoder pulses
    """
    
    def __init__(self):
        super().__init__('encoder_reader')
        
        # Create publishers for encoder counts
        self.left_encoder_pub = self.create_publisher(Int32, 'left_encoder_count', 10)
        self.right_encoder_pub = self.create_publisher(Int32, 'right_encoder_count', 10)
        
        # GPIO setup
        self.encoder_left_pin = 35  # GPIO pin for left encoder
        self.encoder_right_pin = 37  # GPIO pin for right encoder
        
        # Initialize encoder counts
        self.encoder_left_count = 0
        self.encoder_right_count = 0
        
        # Previous states for edge detection
        self.prev_left_state = None
        self.prev_right_state = None
        
        # Setup GPIO
        try:
            # Clean up any existing GPIO setup
            GPIO.cleanup()
            
            # Disable warnings
            GPIO.setwarnings(False)
            
            # Set mode to BCM
            GPIO.setmode(GPIO.BCM)
            self.get_logger().info("GPIO mode set to BCM")
            
            # Setup pins
            GPIO.setup(self.encoder_left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.encoder_right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.get_logger().info("GPIO pins setup complete")
            
            # Get initial states
            self.prev_left_state = GPIO.input(self.encoder_left_pin)
            self.prev_right_state = GPIO.input(self.encoder_right_pin)
            self.get_logger().info(f"Initial pin states - Left: {self.prev_left_state}, Right: {self.prev_right_state}")
            
        except Exception as e:
            self.get_logger().error(f"Error during GPIO setup: {str(e)}")
            raise
        
        # Create timer for polling encoders and publishing values
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1kHz polling rate
        
        self.get_logger().info("EncoderReader node has started.")
        self.get_logger().info(f"Listening on GPIO pins {self.encoder_left_pin} and {self.encoder_right_pin}.")
    
    def timer_callback(self):
        """Poll encoders and publish counts."""
        try:
            # Read current states
            current_left = GPIO.input(self.encoder_left_pin)
            current_right = GPIO.input(self.encoder_right_pin)
            
            # Check for state changes
            if current_left != self.prev_left_state:
                self.encoder_left_count += 1
                self.prev_left_state = current_left
            
            if current_right != self.prev_right_state:
                self.encoder_right_count += 1
                self.prev_right_state = current_right
            
            # Publish counts
            left_msg = Int32()
            right_msg = Int32()
            
            left_msg.data = self.encoder_left_count
            right_msg.data = self.encoder_right_count
            
            self.left_encoder_pub.publish(left_msg)
            self.right_encoder_pub.publish(right_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")
    
    def close(self):
        """Cleanup GPIO on node destruction."""
        try:
            GPIO.cleanup()
            self.get_logger().info("GPIO cleanup completed")
        except Exception as e:
            self.get_logger().error(f"Error during GPIO cleanup: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        node.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 