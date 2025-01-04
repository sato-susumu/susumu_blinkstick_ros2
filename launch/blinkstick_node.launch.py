# susumu_blinkstick_ros2/launch/blinkstick_node.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='susumu_blinkstick_ros2',
            executable='blinkstick_node',
            name='blinkstick_node',
            output='screen',
            parameters=[
                {'led_count': 8}
            ]
        )
    ])
