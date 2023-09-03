import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):
        # Get the launch directory
    map_filter_name = LaunchConfiguration('map_filter_name').perform(context)
    map_yaml = map_filter_name + "_mask.yaml"
    map_param = map_filter_name + "_param.yaml"
    mask = os.path.join(get_package_share_directory('sim_worlds2'),
            'maps', map_yaml)
    params_file = os.path.join(get_package_share_directory('sim_worlds2'),
            'config',map_param)

    # Create our own temporary YAML files that include substitutions
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # Parameters
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
        # Make re-written yaml
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])
    
    return [
        start_lifecycle_manager_cmd,
        start_map_server_cmd,
        start_costmap_filter_info_server_cmd
    ]



def generate_launch_description():

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')


    map_name_arg = DeclareLaunchArgument('map_filter_name', default_value='shimizu_keepout', description='Name of the filtered map')
    return LaunchDescription([
        map_name_arg,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        OpaqueFunction(function=launch_setup)
    ])