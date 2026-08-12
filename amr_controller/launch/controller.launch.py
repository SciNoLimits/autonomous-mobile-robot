from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the controller.yaml file
    config_file_path = os.path.join(
        get_package_share_directory('amr_controller'),
        'config',
        'controller.yaml'
    )

    # Create a Node action for the controller node
    controller_node = Node(
        package='amr_controller',
        executable='amr_controller',
        name='controller',
        output='screen',
        parameters=[config_file_path]
    )

    # Create and return the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(controller_node)
    return ld
