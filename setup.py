from setuptools import setup
import os
from glob import glob

package_name = 'sensor_fusion_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Sensor Fusion Project with ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_imu_pub = sensor_fusion_project.fake_imu_publisher:main',
            'fake_odom_pub = sensor_fusion_project.fake_odom_publisher:main',
        ],
    },
)

