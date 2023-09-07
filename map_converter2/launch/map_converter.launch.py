import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
def generate_launch_description():
    map_save_path = "/home/user/humble_ws/src/nav2_kit/sim_worlds2/models/map_aksk/meshes"
    map_file = os.path.join(get_package_share_directory('sim_worlds2'),
            'maps',
            'aksk.yaml')
    
    map_server =         Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            respawn=True,
            output='screen',
            parameters=[
                        {'use_sim_time': True},
                        {'topic_name': "map"},
                        {'frame_id': "map"},
                        {'yaml_filename': map_file}]
        )


    map_converter =  Node(
            package='map_converter2',
            executable='map_converter',
            name='map_converter',
            parameters=[
                {'map_topic': 'map'},
                {'map_name': 'aksk'},
                {'mesh_type': 'stl'},
                {'export_dir': map_save_path},
                {'occupied_thresh': 1},
                {'box_height': 4.0},
            ],
            output='screen',
        )

    delayed_map_converter =   RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=map_converter,
            on_exit=[map_server],
        )
    )
    return LaunchDescription([
        map_converter,
        # map_server,
        # delayed_map_converter,
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_map_server',
        #     output='screen',
        #     parameters=[{'use_sim_time': True},
        #                 {'autostart': True},
        #                 {'bond_timeout': 0.0},
        #                 {'node_names': ['map_server']}]
        # ),


    ])
