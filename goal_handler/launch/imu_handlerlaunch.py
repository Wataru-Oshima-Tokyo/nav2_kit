import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    home_directory = os.path.expanduser('~')
    print(home_directory)
    
    imu_pose_node =  Node(
            package='goal_handler',
            executable='imu_pose_publisher',
            name='imu_pose_publisher',
            respawn=True,
            output='screen'
        )
    map_to_imu_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        )


    return LaunchDescription([
        imu_pose_node,
        map_to_imu_node
    ])
