import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction

def generate_launch_description():
    map_save_path = "/home/wataru/humble_ws/src/nav2_kit/sim_worlds2/models/ts_1st/meshes"
    map_file = os.path.join(get_package_share_directory('sim_worlds2'),
            'maps',
            'ts_1st.yaml')
    
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

    map_life_cycle_node =     Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': ['map_server']}]
        )

    map_converter =  Node(
            package='map_converter2',
            executable='map_converter',
            name='map_converter',
            parameters=[
                {'map_topic': 'map'},
                {'map_name': 'ts_1st'},
                {'mesh_type': 'stl'},
                {'export_dir': map_save_path},
                {'occupied_thresh': 1},
                {'box_height': 4.0},
            ],
            output='screen',
        )
    timer_map_life_cycle_node = TimerAction(period=2.0, actions=[map_life_cycle_node])

    # delayed_map_converter =   RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=map_converter,
    #         on_start=[map_server],
    #     )
    # )
    return LaunchDescription([
        map_converter,
        map_server,
        timer_map_life_cycle_node
    ])
