#!/usr/bin/env python3

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    
    right_image_topic = LaunchConfiguration('right_image_topic')
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_camera_info_topic = LaunchConfiguration('right_camera_info_topic')
    left_camera_info_topic = LaunchConfiguration('left_camera_info_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([    
    
        DeclareLaunchArgument('right_image_topic', default_value='zed2/right/image_rect_color/compressed'),
        DeclareLaunchArgument('left_image_topic', default_value='zed2/left/image_rect_color/compressed'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='zed2/right/camera_info'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='zed2/left/camera_info'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
                        
        Node(package='ros1_bridge',
             executable='comp_image_bridge',
             name='zed2_left_camera_image_bridge',
             output='screen',
             parameters=[{'image_topic': left_image_topic,
                         'use_sim_time': use_sim_time}]),
                         
        Node(package='ros1_bridge',
             executable='comp_image_bridge',
             name='zed2_right_camera_image_bridge',
             output='screen',
             parameters=[{'image_topic': right_image_topic,
                         'use_sim_time': use_sim_time}]),
                         
        Node(package='ros1_bridge',
             executable='camera_info_bridge',
             name='zed2_left_camera_info_bridge',
             output='screen',
             parameters=[{'topic': left_camera_info_topic,
                         'use_sim_time': use_sim_time}]),
                         
        Node(package='ros1_bridge',
             executable='camera_info_bridge',
             name='zed2_right_camera_info_bridge',
             output='screen',
             parameters=[{'topic': right_camera_info_topic,
                         'use_sim_time': use_sim_time}]),
                         
        ])