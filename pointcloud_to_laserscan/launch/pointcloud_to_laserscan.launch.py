#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/velodyne_points',
        description='Input PointCloud2 topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', 
        default_value='/scan_2d',
        description='Output LaserScan topic'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for the laser scan'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('pointcloud_to_laserscan'),
            'config',
            'pointcloud_to_laserscan_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    # PointCloud to LaserScan converter node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg, 
        frame_id_arg,
        config_file_arg,
        pointcloud_to_laserscan_node,
    ])
