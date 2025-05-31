from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='astabot',
            executable='astabot',
            output='screen',
            emulate_tty=True
        )
    ])
