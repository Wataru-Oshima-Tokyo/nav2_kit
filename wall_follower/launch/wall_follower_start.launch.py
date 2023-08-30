from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'velodyne', 'cloud']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'velodyne', 'cloud2']
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan_for_move')],
            parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -math.pi / 6,
                'angle_max': math.pi / 6,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': False,
                'inf_epsilon': 4.0
            }],
            name='pointcloud_to_laserscan_for_move'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'cloud2',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 5.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': False,
                'inf_epsilon': 4.0
            }],
            name='pointcloud_to_laserscan_for_amcl'
        ),        
        Node(
            package='wall_follower',
            executable='wall_follower_node',
            name='wall_follower_node',
            output="screen"
        )
    ])
