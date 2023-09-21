import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    map_name = LaunchConfiguration('map_name').perform(context)
    map_name = map_name + ".yaml"
    map_file_path = os.path.join(get_package_share_directory('sim_worlds2'),
        'maps',
         map_name)      

    map_config = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', 'map_config.yaml')
    map_handler =  Node(
            package='map_handler',
            executable='map_position_change_node',
            name='map_position_change_node',
            respawn=True,
            output='screen',
            parameters=[{"map_file_path": map_file_path}]
        )

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            respawn=True,
            output='screen',
            parameters=[map_config,
                        {'topic_name': "map"},
                        {'frame_id': "map"},
                        {'yaml_filename': map_file_path}]
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

    return [
        map_handler,
        map_server_node,
        map_life_cycle_node
    ]


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument('map_name', default_value='aksk.yaml', description='Name of the map')

    return LaunchDescription([
        map_name_arg,
        OpaqueFunction(function=launch_setup)
    ])