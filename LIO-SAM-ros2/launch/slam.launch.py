import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,RegisterEventHandler,GroupAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression,Command,PathJoinSubstitution
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    lio_parameter_file = LaunchConfiguration('lio_parameter_file')

    params_declare = DeclareLaunchArgument(
        'lio_parameter_file',
        default_value=os.path.join(
            share_dir, 'config', 'world_map.yaml'),
        description='FPath to the ROS2 parameters file to use.')
    

    velodyne_odom_to_base_link_node =  Node(
            package='fake_odom',
            executable='fake_odom_broadcaster',
            name='fake_odom_to_base_link',
        )


    map_to_odom_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        )

    lio_sam_nodes = GroupAction(
        actions=[
            Node(
                package='lio_sam',
                executable='lio_sam_imuPreintegration',
                name='lio_sam_imuPreintegration',
                parameters=[lio_parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_imageProjection',
                name='lio_sam_imageProjection',
                parameters=[lio_parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_featureExtraction',
                name='lio_sam_featureExtraction',
                parameters=[lio_parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_mapOptimization',
                name='lio_sam_mapOptimization',
                parameters=[lio_parameter_file],
                output='screen'
            ),
        ]
    )


    delayed_lio_sam_server =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=map_to_odom_node,
            on_start=[lio_sam_nodes],
        )
    )

    delayed_fake_odom =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=map_to_odom_node,
            on_start=[velodyne_odom_to_base_link_node],
        )
    )


    return LaunchDescription([
        params_declare,
        map_to_odom_node,
        delayed_lio_sam_server,
        delayed_fake_odom,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map']
        ),
    ])
