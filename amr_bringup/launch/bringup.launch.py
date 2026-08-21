#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    amr_navigation_share_dir = get_package_share_directory(
        'amr_navigation'
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                amr_navigation_share_dir,
                'launch',
                'amr.launch.py'
            )
        )
    )

    mission_control_gui = Node(
        package='amr_navigation',
        executable='mission_control_gui',
        name='mission_control_gui',
        output='screen'
    )

    return LaunchDescription([
        navigation_launch,
        mission_control_gui,
    ])

