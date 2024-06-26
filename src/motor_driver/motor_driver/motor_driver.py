#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        self.declare_parameter('wheel_base', 0.26)
        self.declare_parameter('wheel_radius', 0.069)

        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Motor control GPIO pins
        self.motor1_pins = (16, 13)  # IN1, IN2 for Motor 1
        self.motor2_pins = (19, 26)  # IN1, IN2 for Motor 2

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor1_pins, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motor2_pins, GPIO.OUT, initial=GPIO.LOW)

        self.motor1_pwm1 = GPIO.PWM(self.motor1_pins[0], 200)  # 200 Hz
        self.motor1_pwm2 = GPIO.PWM(self.motor1_pins[1], 200)  # 200 Hz
        self.motor2_pwm1 = GPIO.PWM(self.motor2_pins[0], 200)  # 200 Hz
        self.motor2_pwm2 = GPIO.PWM(self.motor2_pins[1], 200)  # 200 Hz
        self.motor1_pwm1.start(0)
        self.motor1_pwm2.start(0)
        self.motor2_pwm1.start(0)
        self.motor2_pwm2.start(0)

        # Subscribe to twist command topic
        self.subscription_twist = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

        self.get_logger().info("Motor driver node has started.")

    def set_motor_velocity(self, pwm1, pwm2, velocity):
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
        self.get_logger().info(f"Set motor velocity to {velocity} with duty cycle {duty_cycle}")

    def twist_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate wheel velocities based on differential drive kinematics
        v_left = (linear_velocity - (angular_velocity * self.wheel_base / 2)) / self.wheel_radius
        v_right = (linear_velocity + (angular_velocity * self.wheel_base / 2)) / self.wheel_radius

        # Log the velocities for debugging
        self.get_logger().info(f"Twist received: linear={linear_velocity}, angular={angular_velocity}")
        self.get_logger().info(f"Calculated wheel velocities: left={v_left}, right={v_right}")

        # Send wheel velocities to the motor control functions
        self.set_motor_velocity(self.motor1_pwm1, self.motor1_pwm2, v_left)
        self.set_motor_velocity(self.motor2_pwm1, self.motor2_pwm2, v_right)

    def close(self):
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

if __name__ == "__main__":
    main()
