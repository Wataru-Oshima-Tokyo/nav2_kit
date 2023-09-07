import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    share_dir = get_package_share_directory('robot_navigation')
    return [

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share_dir, 'launch', str(lidar_type+'_to_scan.launch.py'))
            ),
        ),
        # Node(
        #     package='robot_navigation',
        #     executable='safety_relay',
        #     name='safety_relay',
        #     output='screen'
        # )
    ]

def generate_launch_description():
    # Declare arguments

    lidar_type_arg = DeclareLaunchArgument('lidar_type', default_value='velodyne', description='Type of lidar')
    

    return LaunchDescription([
        lidar_type_arg,
        OpaqueFunction(function=launch_setup)
    ])
