from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    waypoint_server_node = Node(
        package='amr_navigation',
        executable='waypoint_server',
        name='waypoint_server',
        output='screen',
    )
    
    patrol_server_node = Node(
        package='amr_navigation',
        executable='patrol_server',
        name='patrol_server',
        output='screen',
    )
    
    ld.add_action(waypoint_server_node)
    ld.add_action(patrol_server_node)
    
    return ld
    
    