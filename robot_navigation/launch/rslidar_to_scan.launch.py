from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math
import os
from ament_index_python.packages import get_package_share_directory
    
def generate_launch_description():

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_to_cloud',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'rslidar', 'cloud']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_to_cloud2',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'rslidar', 'cloud2']
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/rslidar_points'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 8.0,
                'use_inf': False,
                'inf_epsilon': 4.0
            }],
            name='pointcloud_to_laserscan_for_move'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/rslidar_points'),
                        ('scan', '/scan_for_amcl')],
            parameters=[{
                'target_frame': 'cloud2',
                'transform_tolerance': 0.01,
                'min_height': 3.0,
                'max_height': 15.0,
                'angle_min': -math.pi,  # -M_PI/2
                'angle_max': math.pi,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 50.0,
                'use_inf': False,
                'inf_epsilon': 4.0
            }],
            name='pointcloud_to_laserscan_for_amcl'
        ),
        
    ])
