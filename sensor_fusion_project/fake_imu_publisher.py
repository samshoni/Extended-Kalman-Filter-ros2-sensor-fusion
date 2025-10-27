#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class FakeImuPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)  # 50Hz
        self.counter = 0.0
        self.get_logger().info('Fake IMU Publisher Started')

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Simulate angular velocity with some noise
        msg.angular_velocity.x = 0.01 * math.sin(self.counter)
        msg.angular_velocity.y = 0.01 * math.cos(self.counter)
        msg.angular_velocity.z = 0.05 * math.sin(self.counter * 0.5)
        
        # Simulate linear acceleration
        msg.linear_acceleration.x = 0.1 * math.cos(self.counter)
        msg.linear_acceleration.y = 0.1 * math.sin(self.counter)
        msg.linear_acceleration.z = 9.81  # gravity
        
        # Set covariance (uncertainty) values
        msg.angular_velocity_covariance = [0.001] * 9
        msg.linear_acceleration_covariance = [0.001] * 9
        
        self.publisher_.publish(msg)
        self.counter += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = FakeImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

