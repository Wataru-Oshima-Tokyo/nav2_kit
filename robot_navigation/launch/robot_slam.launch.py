# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # this_package = FindPackageShare('robot_navigation')

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('robot_navigation'), 'config', 'slam.rviz']
    )


    default_params_file_path = PathJoinSubstitution(
        [FindPackageShare('robot_navigation'), 'param', 'slam.yaml']
    )

    # static_map_to_odom_node =  Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='map_to_odom',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    # )

    
    static_map_to_dlio_odom_link =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_laser_link',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'dlio_odom']
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),


        DeclareLaunchArgument(
            name='slam_params_file',
            default_value=default_params_file_path,
            description='Navigation2 slam params file'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'slam_params_file': LaunchConfiguration("slam_params_file")
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        ),
        # static_base_footprint_to_laser_link
        # static_map_to_dlio_odom_link
    ])