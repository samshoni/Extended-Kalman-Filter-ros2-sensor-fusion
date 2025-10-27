#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fusion_project')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    return LaunchDescription([
        # Fake IMU Publisher
        Node(
            package='sensor_fusion_project',
            executable='fake_imu_pub',
            name='fake_imu_publisher',
            output='screen'
        ),
        
        # Fake Odometry Publisher
        Node(
            package='sensor_fusion_project',
            executable='fake_odom_pub',
            name='fake_odom_publisher',
            output='screen'
        ),
        
        # Robot Localization EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        )
    ])

