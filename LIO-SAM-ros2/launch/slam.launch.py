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
    fake_frame = "fake_velodyne_link"
    share_dir = get_package_share_directory('lio_sam')
    lio_parameter_file = LaunchConfiguration('lio_parameter_file')



    
    params_declare = DeclareLaunchArgument(
        'lio_parameter_file',
        default_value=os.path.join(
            share_dir, 'config', 'world_map.yaml'),
        description='FPath to the ROS2 parameters file to use.')
    
    # velodyne_to_fake_velodyne =  Node(
    #     package='fake_frame',
    #     executable='fake_point_clouds',
    #     name='velodyne_to_fake_velodyne',
    #         parameters=[{'fake_frame_id': fake_frame},
    #                     {'target_topic': "points_raw"}]   
    # )

    velodyne_to_base_link =  Node(
        package='fake_frame',
        executable='fake_dynamic_tf_broadcaster',
        name='map_to_odom',
            parameters=[{'parent_link': "velodyne"},
                        {'child_link': "base_link"},
                        {"use_sim_time": True}]
        
    )



    # base_link_to_fake_velodyne =  Node(
    #     package='fake_frame',
    #     executable='fake_dynamic_tf_broadcaster',
    #     name='map_to_odom',
    #         parameters=[{'parent_link': "velodyne"},
    #                     {'child_link': fake_frame},
    #                     {"use_sim_time": True}]
        
    # )
    # world_to_map_node =  Node(
    #     package='fake_frame',
    #     executable='fake_dynamic_tf_broadcaster',
    #     name='world_to_map',
    #         parameters=[{'parent_link': "world"},
    #                     {'child_link': "map"},
    #                     {"use_sim_time": True}]
    # )
    # static_marker_to_odom_node =  Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='map_to_odom',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'marker', 'odom']
    #     )

    static_world_to_map_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map']
    )
    
    # base_link_fake_velodyne =  Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='base_link_to_fake_velodyne',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', fake_frame]
    #     )

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

    # delayed_marker_to_odom = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=static_world_to_map_node,
    #         on_start=[static_marker_to_odom_node],
    #     )
    # )

    delayed_lio_sam_server =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[lio_sam_nodes],
        )
    )

    delayed_fake_odom =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=static_world_to_map_node,
            on_start=[velodyne_to_base_link],
        )
    )

    # delayed_fake_velodyne =   RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=velodyne_to_base_link,
    #         on_start=[base_link_to_fake_velodyne],
    #     )
    # )


    return LaunchDescription([
        params_declare,
        static_world_to_map_node,
        delayed_lio_sam_server,
        delayed_fake_odom,
        # velodyne_to_fake_velodyne,
        # delayed_fake_velodyne,
    ])
