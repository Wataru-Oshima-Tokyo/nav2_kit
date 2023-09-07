import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
import distro

def get_ros2_distro():
    ubuntu_version = distro.version()
    if ubuntu_version == '20.04':
        return 'foxy'
    elif ubuntu_version == '22.04':
        return 'humble'
    else:
        return 'unknown'


def launch_setup(context, *args, **kwargs):
    ros2_distro = get_ros2_distro()
    print('ROS 2 Distro:', ros2_distro)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_name = LaunchConfiguration('map_name').perform(context)

    param_dir = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param','robot.yaml')
    map_file = os.path.join(get_package_share_directory('sim_worlds2'),
        'maps',
         map_name)          
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                }.items(),
        ),
    ]


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument('map_name', default_value='akskR3.yaml', description='Name of the map')
    return LaunchDescription([
        map_name_arg,
        OpaqueFunction(function=launch_setup)

    ])
