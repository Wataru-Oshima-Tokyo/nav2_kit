from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math
import os
from ament_index_python.packages import get_package_share_directory
    
def generate_launch_description():
    theta_min_deg = -60
    theta_max_deg = 60

    theta_min_rad = theta_min_deg * math.pi / 180
    theta_max_rad = theta_max_deg * math.pi / 180
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 -0.8 0.0 0.0 0.0 velodyne cloud_link'.split(' '),
            output='screen'
            ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan')],
            parameters=[{
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 10.0,
                'use_inf': False,
                'inf_epsilon': 4.0
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan_for_amcl')],
            parameters=[{
                'min_height': 1.0,
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
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan_for_move')],
            parameters=[{
                'min_height': -0.4,
                'max_height': 3.0,
                'angle_min': theta_min_rad,
                'angle_max': theta_max_rad,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 7.0,
                'use_inf': False,
                'inf_epsilon': 4.0
            }],
            name='pointcloud_to_laserscan_for_move'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='laserscan_to_pointcloud_node',
            remappings=[('scan_in', '/scan_for_move'),
                        ('cloud', '/obstacle_points_raw')],
            parameters=[{
                'target_frame': "cloud_link",
                'transform_tolerance': 0.01,
            }],
            name='scan_to_pc_for_object_detection'
        ),
        

    ])
