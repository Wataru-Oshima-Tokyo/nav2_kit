import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share_dir = get_package_share_directory("map_handler")
    pcd_file_path = os.path.join(share_dir, 'pcd')

    node_params = [
        {'folder_path': pcd_file_path},
        {'pcd_filename': 'dlio_map.pcd'},
        {'map_rosbag_topic': 'grid_map'},
        {'output_grid_map': 'elevation_map.bag'},
        {'map_frame': 'map'},
        {'map_layer_name': 'elevation'},
        {'prefix': ''},
        {'set_verbosity_to_debug': False}
    ]

    pcl_loader_node = Node(
        package='grid_map_pcl',
        executable='grid_map_pcl_loader_node',
        name='grid_map_pcl_loader_node',
        output='screen',
        parameters=node_params
    )

    ld = LaunchDescription()

    ld.add_action(pcl_loader_node)

    return ld