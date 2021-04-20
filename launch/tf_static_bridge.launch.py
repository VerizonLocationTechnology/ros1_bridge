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
    
    topic = LaunchConfiguration('tf_static_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([    
    
        DeclareLaunchArgument('tf_static_topic', default_value='tf_static'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
                         
        Node(package='ros1_bridge',
             executable='tf_static_bridge',
             name='tf_static_bridge',
             output='screen',
             parameters=[{'topic': topic,
                         'use_sim_time': use_sim_time}]),
                         
        ])