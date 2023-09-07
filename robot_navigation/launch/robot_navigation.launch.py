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
    robot_name = LaunchConfiguration('robot_name').perform(context)

    default_nav_to_pose_bt_xml_robot = os.path.join(get_package_share_directory(
    'robot_navigation'), 'config', robot_name, ros2_distro, 'nav_thr_poses.xml')
    param_dir = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', robot_name, ros2_distro, 'robot.yaml')
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
                'params_file': param_dir
                }.items(),
        ),
    ]


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument('map_name', default_value='akskR3.yaml', description='Name of the map')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='limo_ackermann', description='Robot Name')
    # map_to_odom_node =  Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='map_to_odom',
    #         arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    #     )
    return LaunchDescription([
        map_name_arg,
        robot_name_arg,
        # map_to_odom_node,
        OpaqueFunction(function=launch_setup)

    ])
