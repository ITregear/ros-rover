import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Parameters
        self.spin_slip_factor = 1.76

        self.declare_parameter('wheel_base', 0.26)  # Distance between wheels
        self.declare_parameter('wheel_radius', 0.069)  # Radius of the wheels
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Subscriptions
        self.create_subscription(Float64, 'motor1_velocity', self.motor1_callback, 10)
        self.create_subscription(Float64, 'motor2_velocity', self.motor2_callback, 10)

        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize variables
        self.motor1_velocity = 0.0
        self.motor2_velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Timer to update odometry
        self.create_timer(0.1, self.update_odometry)

    def motor1_callback(self, msg):
        self.motor1_velocity = msg.data

    def motor2_callback(self, msg):
        self.motor2_velocity = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Compute linear and angular velocities
        v1 = self.motor1_velocity * self.wheel_radius
        v2 = self.motor2_velocity * self.wheel_radius
        v = (v1 + v2) / 2.0
        omega = (v2 - v1) / self.wheel_base

        # Update robot pose
        self.x += v * cos(self.theta) * dt
        self.y += v * sin(self.theta) * dt
        self.theta += omega * dt

        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = cos(self.theta / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_publisher.publish(odom)

        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = odom.pose.pose.orientation.z
        transform.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
