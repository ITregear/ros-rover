from setuptools import setup
import os
from glob import glob

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivantregear@gmail.com',
    description='ROS2 package for controlling a differential drive rover with motor drivers and encoders',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = rover_control.motor_driver:main',
            'encoder_publisher = rover_control.encoder_publisher:main',
            'velocity_controller = rover_control.velocity_controller:main',
        ],
    },
)
