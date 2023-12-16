
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_file = LaunchConfiguration('rviz_file').perform(context)
    rviz_config_path = os.path.join(get_package_share_directory('robot_navigation'),'config', rviz_config_file) 

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_path])

    return [
        start_rviz_cmd
    ]
 
def generate_launch_description():
 
  
  declare_rviz_file_cmd = DeclareLaunchArgument(
    name='rviz_file',
    default_value='limo_nav.rviz',
    description='rviz file name')

  

  return LaunchDescription([
      declare_rviz_file_cmd,
      OpaqueFunction(function=launch_setup)

  ])
