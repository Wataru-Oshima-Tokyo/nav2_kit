#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration ,Command,PathJoinSubstitution,FindExecutable
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import lifecycle_msgs.msg
import os



def generate_launch_description():
    share_dir = get_package_share_directory('robot_navigation')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'config', 'rslidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')
    driver_node = Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen',
            )
    tf_ros= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0.23', '0', '0', '0', '1', 'base_link', 'rslidar']
            )
    
    return LaunchDescription([
        params_declare,
        driver_node,
        tf_ros,
    ])
