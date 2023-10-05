from launch import LaunchDescription
from launch_ros.actions import Node
import math


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_trim',
            # namespace='point_clouds_trim',
            executable='point_clouds_trim',
            name='trimmer',
            parameters=[{
                'topic_name': "trimmed_points",  
                'min_height': -1.0,
                'max_height': 2.0,
                'angle_min': -60.0,
                'angle_max': 60.0,
                'range_min': 0.1,
                'range_max': 5.0,
            }],
        )
    ])
