import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,RegisterEventHandler,GroupAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression,Command,PathJoinSubstitution
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    sim = LaunchConfiguration('sim')
    navigation_param = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', "slam.yaml")

    sim_declare =  DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
    )

    
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )
    navigation_node =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': navigation_param
            }.items()
    )




    return LaunchDescription([
        sim_declare,
        navigation_node
        # delayed_lio_sam_server,
    ])
