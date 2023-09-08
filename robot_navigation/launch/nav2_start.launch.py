import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
    robot_name = LaunchConfiguration('robot_name').perform(context)
    param_file = robot_name +".yaml"
    param_dir = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', param_file)
    map_file = os.path.join(get_package_share_directory('sim_worlds2'),
        'maps',
         map_name)          
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_navigation'), 'launch', 'navigate.launch.py']
    )
    if (robot_name is not "champ"):
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
    else:
        return [
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': map_file,
                'params_file': param_dir,
                'sim': use_sim_time,
                'rviz': False
            }.items()
        )
        ]


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument('map_name', default_value='akskR3.yaml', description='Name of the map')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='diffbot', description='Name of the map')

    return LaunchDescription([
        map_name_arg,
        robot_name_arg,
        OpaqueFunction(function=launch_setup)

    ])
