#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the amr_controller and amr_navigation package
    amr_controller_share_dir = get_package_share_directory('amr_controller')
    amr_navigation_share_dir = get_package_share_directory('amr_navigation')

    # Create an IncludeLaunchDescription action for the launch.py file
    turtlebot3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_gazebo"),
                "launch",
                "empty_world.launch.py",
            )
        )
    )
    
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_controller_share_dir, 'launch', 'controller.launch.py')
        )
    )
    
    patrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_navigation_share_dir, 'launch', 'patrol.launch.py')
        )
    )

    # Create and return the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(turtlebot3_launch)
    ld.add_action(controller_launch)
    ld.add_action(patrol_launch)
    return ld