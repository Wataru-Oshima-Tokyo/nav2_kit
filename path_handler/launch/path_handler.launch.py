import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_handler',
            executable='path_handler',
            name='path_handler_node',
            output='screen'
        )
    ])
