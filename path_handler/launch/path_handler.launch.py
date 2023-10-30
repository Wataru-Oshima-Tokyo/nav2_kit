import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    share_dir = get_package_share_directory('path_handler')
    path_file_path = os.path.join(share_dir,'path')

    # Ensure the directory exists
    if not os.path.exists(path_file_path):
        os.makedirs(path_file_path)


    return LaunchDescription([
        Node(
            package='path_handler',
            executable='path_handler',
            name='path_handler_node',
            output='screen',
            parameters=[{"map_file_path": path_file_path}]
        )
    ])
