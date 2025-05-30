#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

import lifecycle_msgs.msg
import os

USE_RVIZ = os.environ['USE_RVIZ']

def generate_launch_description():

    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')
                     
    driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		#设置激光数据topic名称
                                output='log',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver_dir],
                                )


    rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz', 'astabot.rviz')

    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        condition=IfCondition(USE_RVIZ),
        output='log')

    return LaunchDescription([
        driver_node,
        rviz_node,
    ])

