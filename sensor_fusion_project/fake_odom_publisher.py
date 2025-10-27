#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.timer = self.create_timer(0.02, self.publish_odom)  # 50Hz
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.counter = 0.0
        self.get_logger().info('Fake Odometry Publisher Started')

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Simulate robot moving in circular pattern
        linear_vel = 0.2  # m/s
        angular_vel = 0.1  # rad/s
        
        dt = 0.02
        self.x += linear_vel * math.cos(self.theta) * dt
        self.y += linear_vel * math.sin(self.theta) * dt
        self.theta += angular_vel * dt
        
        # Set position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert yaw to quaternion)
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Set velocity
        msg.twist.twist.linear.x = linear_vel
        msg.twist.twist.angular.z = angular_vel
        
        # Set covariance
        msg.pose.covariance = [0.001] * 36
        msg.twist.covariance = [0.001] * 36
        
        self.publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

