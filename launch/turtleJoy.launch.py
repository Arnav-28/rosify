#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Check if we're told to use sim time
    package_name = 'rosify' #<--- CHANGE ME

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
  
    turtleJoy_exe_node = Node(
            package='rosify',  # Replace with your package name
            executable='turtleJoy.py',  # Replace with your node's executable name
        )
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )


    # Launch!
    return LaunchDescription([
        turtlesim_node,
        turtleJoy_exe_node
    ])
