import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    map_name = LaunchConfiguration('map_name').perform(context)
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    robot_amcl_config = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', 'robot_amcl.yaml')
    robot_nav_pkg = FindPackageShare(package='robot_navigation').find('robot_navigation')   
    
    map_file = os.path.join(get_package_share_directory('sim_worlds2'),
        'maps',
         map_name)

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_nav_pkg, 'launch', str(lidar_type+'_to_scan.launch.py'))
            ),
        ),
        Node(
            package='robot_navigation',
            executable='safety_net',
            name='emergency_stop',
            output='screen'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'topic_name': "map"},
                        {'frame_id': "map"},
                        {'yaml_filename': map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[robot_amcl_config],
            remappings=remappings
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
    ]

def generate_launch_description():
    # Declare arguments

    map_name_arg = DeclareLaunchArgument('map_name', default_value='sh.yaml', description='Name of the map')
    lidar_type_arg = DeclareLaunchArgument('lidar_type', default_value='velodyne', description='Type of lidar')
    

    return LaunchDescription([
        map_name_arg,
        lidar_type_arg,
        OpaqueFunction(function=launch_setup)
    ])
