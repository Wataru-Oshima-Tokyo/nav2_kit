import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument, ExecuteProcess,RegisterEventHandler,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration,PythonExpression, Command
from launch.event_handlers import OnProcessExit

def launch_setup(context, *args, **kwargs):
    home_directory = os.path.expanduser('~')
    print(home_directory)
    goal_save_path = home_directory + "/humble_ws/src/nav2_kit/goal_handler/goals/"
    cmd_vel_topic = "/diff_cont/cmd_vel_unstamped"
    map_name = LaunchConfiguration('map_name').perform(context)
    goal_name = map_name + "_goals.yaml"
    share_dir = get_package_share_directory('path_handler')
    path_file_path = os.path.join(share_dir,'path')

    # Ensure the directory exists
    if not os.path.exists(path_file_path):
        os.makedirs(path_file_path)

    path_handler =  Node(
            package='path_handler',
            executable='path_handler',
            name='path_handler_node',
            output='screen',
            parameters=[{"trajectory_file_path": path_file_path}]
        )


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
            parameters=[{"goals_file": goal_save_path}, {"cmd_vel": cmd_vel_topic}, {"goal_name": goal_name}]

        )

    return [
        goal_handler,
        goal_publisher,
        path_handler
    ]
def generate_launch_description():

    map_name_declare =  DeclareLaunchArgument(
            name='map_name', 
            default_value='sh',
            description='Enable use_sime_time to true'
    )



    return LaunchDescription([

        map_name_declare,
        OpaqueFunction(function=launch_setup)
    ])
