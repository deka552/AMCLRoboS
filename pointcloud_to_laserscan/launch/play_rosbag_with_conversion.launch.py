#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='/stand2/stand2_0.db3',
        description='Path to the rosbag file'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/velodyne_points',
        description='Input PointCloud2 topic from rosbag'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/scan_2d', 
        description='Output LaserScan topic'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop the rosbag playback'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate'
    )
    
    # ROS bag play command
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('rosbag_path'),
            '--loop' if LaunchConfiguration('loop') == 'true' else '',
            '--rate', LaunchConfiguration('rate'),
        ],
        output='screen',
        name='rosbag_play'
    )
    
    # Include the pointcloud to laserscan converter
    pointcloud_converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pointcloud_to_laserscan'),
                'launch',
                'pointcloud_to_laserscan.launch.py'
            ])
        ]),
        launch_arguments={
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
        }.items()
    )
    
    return LaunchDescription([
        rosbag_path_arg,
        input_topic_arg,
        output_topic_arg,
        loop_arg,
        rate_arg,
        rosbag_play,
        pointcloud_converter_launch,
    ])
