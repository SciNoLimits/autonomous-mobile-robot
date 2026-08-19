#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # MANDATORY FOR ADDING BRIDGES/NODES


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    amr_simulation_dir = get_package_share_directory('amr_simulation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        amr_simulation_dir,
        'worlds',
        'amr_world.sdf'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # ADDED: This bridge node maps communication pipes between Gazebo and ROS 2
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model]",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry]",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V]",
            "/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist]",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU]",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan]",
        ],
    )


    set_env_vars_tb3_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))
    
    set_env_vars_amr_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                amr_simulation_dir,
                'worlds',
                'models'
            )
        )

    ld = LaunchDescription()

    # Environmental setups MUST happen first
    ld.add_action(set_env_vars_amr_resources)
    ld.add_action(set_env_vars_tb3_resources)
    
    # Launch servers and UI clients
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    # Spawners and data bridges
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(ros_gz_bridge_node)  # REGISTER THE BRIDGE HERE

    return ld
