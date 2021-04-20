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

    launch_dir = os.path.join(
        get_package_share_directory('ros1_bridge'),
        'launch')
        
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    
    declare_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='jackal',
        description='Namespace')
    
    bridges_cmd_group = GroupAction([
        PushRosNamespace(
            namespace=namespace),
                
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir,'tf_static_bridge.launch.py')),
                launch_arguments={'use_sim_time' : use_sim_time,}.items()),
                
        Node(package='ros1_bridge',
             executable='robot_bridge',
             name='robot_bridge',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),   
    
    ])

        
        
    return LaunchDescription([
        declare_sim_time_cmd,
        declare_namespace_cmd,
        bridges_cmd_group,

    ])