from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_path = "/home/wataru/humble_ws/src/nav2_kit/sim_worlds2/models/map_shimizu/meshes/"

    return LaunchDescription([
        Node(
            package='map_converter2',
            executable='map_converter',
            name='map_converter',
            parameters=[
                {'map_topic': 'map'},
                {'map_name': 'shimizu'},
                {'mesh_type': 'stl'},
                {'export_dir': map_path},
                {'occupied_thresh': 1},
                {'box_height': 5.0},
            ],
            output='screen',
        ),
    ])