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
    goal_save_path = home_directory + "/humble_ws/src/nav2_kit/goal_handler/goals/"

    
    goal_handler =  Node(
            package='goal_handler',
            executable='goal_saver_node',
            name='goal_saver_node',
            respawn=True,
            output='screen',
            parameters=[{"output_file": goal_save_path}]
        )
    goal_publisher =  Node(
            package='goal_handler',
            executable='goal_handler_node',
            name='goal_handler_node',
            respawn=True,
            output='screen',
            parameters=[{"goals_file": goal_save_path}]
        )


    return LaunchDescription([
        goal_handler,
        goal_publisher
    ])
