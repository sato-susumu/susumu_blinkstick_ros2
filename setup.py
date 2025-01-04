from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'susumu_blinkstick_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sato Susumu',
    maintainer_email='75652942+sato-susumu@users.noreply.github.com',
    description='BlinkStick LED Animations for ROS2 (Package name: susumu_blinkstick_ros2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blinkstick_node = susumu_blinkstick_ros2.blinkstick_node:main'
        ],
    },
)
