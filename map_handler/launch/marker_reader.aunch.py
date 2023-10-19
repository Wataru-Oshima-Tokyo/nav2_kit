import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Retrieve the path to the apriltag_ros package
    map_name = LaunchConfiguration('map_name').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_topic = "/" + camera_name +"/image_raw"
    camera_info = "/" + camera_name + "/camera_info"
    marker_name = map_name + "_marker.yaml"



    share_dir = get_package_share_directory('map_handler')
    # Define the path to the parameter file
    if not use_sim_time:
        print("DO NOT USE SIM TIME")
        tag_param_file = os.path.join(share_dir,'param', 'tags_36h11.yaml')
    else:
        print("USE SIM TIME")
        tag_param_file = os.path.join(share_dir,'param', 'tags_36h11_sim.yaml')

    marker_param_file = os.path.join(share_dir, 'param', marker_name)
    
    # Load parameters
    with open(marker_param_file, 'r') as f:
        params = yaml.safe_load(f)
        transform = params['static_transform']

    # Set arguments for the static transform publisher from loaded params
    args = [
        str(transform['x']), str(transform['y']), str(transform['z']),
        str(transform['roll']), str(transform['pitch']), str(transform['yaw']),
        '1',  # Quaternion representation of rotation. You might want to calculate it based on Euler angles (roll, pitch, yaw).
        transform['frame_id'], transform['child_frame_id']
    ]

    static_map_to_marker_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_marker',
            arguments=args
    )

    dynamic_marker_to_odom_node =  Node(
        package='map_handler',
        executable='marker_localization',
        name='marker_localization_node',
        respawn=False,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    apriltag_node =         Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('image_rect', camera_topic),
                ('camera_info', camera_info)
            ],
            parameters=[tag_param_file]
        )
    initial_pose_setter_node = Node(
        package='map_handler',
        executable='init_pose_setter',
        name='init_pose_setter_node',
        respawn=False,
        output='screen',
    )
    return [
        static_map_to_marker_node,
        dynamic_marker_to_odom_node,
        apriltag_node,
        initial_pose_setter_node
    ]

def generate_launch_description():
    map_name_arg = DeclareLaunchArgument('map_name', default_value='sh', description='Name of the map')
    camera_name_arg = DeclareLaunchArgument('camera_name', default_value='camera', description='Name of the map')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='use sim time or not')
    return LaunchDescription([
        map_name_arg,
        camera_name_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])

