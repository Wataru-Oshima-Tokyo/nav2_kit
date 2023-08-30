from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_path = "/external/src/"

    return LaunchDescription([
        Node(
            package='map_converter2',
            executable='map_converter',
            name='map_converter',
            parameters=[
                {'map_topic': 'map'},
                {'map_name': 'aksk'},
                {'mesh_type': 'stl'},
                {'export_dir': map_path},
                {'occupied_thresh': 1},
                {'box_height': 7.0},
            ],
            output='screen',
        ),
    ])