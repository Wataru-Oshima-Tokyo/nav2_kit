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
                'cloud_in': "points_raw",
                'cloud_out': "trimmed_points",  
                'min_height': -1.0,
                'max_height': 2.0,
                'angle_min': -60.0,
                'angle_max': 60.0,
                'range_min': 0.1,
                'range_max': 3.0,
                'threshold': 0.01,
                'costmap_interval_': 2000 #milliseconds
            }],
        )
    ])
