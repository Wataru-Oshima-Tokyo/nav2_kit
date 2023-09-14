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
    map_file_path = os.path.join(get_package_share_directory('sim_worlds2'),
        'maps',
         map_name)      

    
    map_handler =  Node(
            package='map_handler',
            executable='load_map_node',
            name='load_map_node',
            respawn=True,
            output='screen',
            parameters=[{"map_file_path": map_file_path}]
        )
    return [
        map_handler
    ]


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument('map_name', default_value='aksk.yaml', description='Name of the map')

    return LaunchDescription([
        map_name_arg,
        OpaqueFunction(function=launch_setup)
    ])
