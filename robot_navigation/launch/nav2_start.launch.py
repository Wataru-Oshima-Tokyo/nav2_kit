import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import distro
from nav2_common.launch import RewrittenYaml

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
    nav_to_pose_xml = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', "nav_to_pose_and_pause_near_goal_obstacle.xml")
    nav_thr_pose_xml = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', "nav_thr_poses.xml")


    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': nav_to_pose_xml,
        'map_subscribe_transient_local': nav_thr_pose_xml}

    configured_params = RewrittenYaml(
            source_file=param_dir,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)



    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
                }.items(),
        ),
    ]



def generate_launch_description():
    map_name_arg = DeclareLaunchArgument('map_name', default_value='akskR3.yaml', description='Name of the map')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='diffbot', description='Name of the robot')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='whether to use rviz')

    return LaunchDescription([
        map_name_arg,
        robot_name_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)

    ])
