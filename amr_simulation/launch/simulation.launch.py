#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    amr_simulation_dir = get_package_share_directory(
        'amr_simulation'
    )

    turtlebot3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    ros_gz_sim_dir = get_package_share_directory(
        'ros_gz_sim'
    )

    world_file = os.path.join(
        amr_simulation_dir,
        'worlds',
        'amr_world.world'
    )

    turtlebot3_launch = os.path.join(
        turtlebot3_gazebo_dir,
        'launch',
        'spawn_turtlebot3.launch.py'
    )

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # -------------------------------------------------
    # Gazebo server
    # -------------------------------------------------

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # -------------------------------------------------
    # Gazebo GUI
    # -------------------------------------------------

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-g -v2 ',
        }.items()
    )

    # -------------------------------------------------
    # TurtleBot3
    # -------------------------------------------------

    turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            turtlebot3_launch
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # -------------------------------------------------
    # TurtleBot3 model resources
    # -------------------------------------------------

    set_gazebo_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            turtlebot3_gazebo_dir,
            'models'
        )
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial X position of TurtleBot3'
        ),

        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial Y position of TurtleBot3'
        ),

        set_gazebo_resource_path,

        gazebo_server,

        gazebo_gui,

        turtlebot3,

    ])