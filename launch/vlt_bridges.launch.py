#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_bridge_path = os.path.join(
        get_package_share_directory('ros1_bridge'),
        'launch', 'robot_bridge.launch.py')
        
    jackal_bridge = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(robot_bridge_path),
                    launch_arguments={'use_sim_time' : 'false',
                                      'namespace' : 'jackal'}.items())
                                      
    dingo_bridge = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(robot_bridge_path),
                    launch_arguments={'use_sim_time' : 'false',
                                      'namespace' : 'dingo'}.items())
                                      
    ghost_bridge = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(robot_bridge_path),
                    launch_arguments={'use_sim_time' : 'false',
                                      'namespace' : 'ghost'}.items())
             
        
    return LaunchDescription([
        jackal_bridge,
        dingo_bridge,
        ghost_bridge,
    ])