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
    parameter_file = LaunchConfiguration('parameter_file')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    sim = LaunchConfiguration('sim')
    navigation_param = os.path.join(get_package_share_directory(
        'robot_navigation'), 'param', "slam.yaml")

    sim_declare =  DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
    )

    params_declare = DeclareLaunchArgument(
        'parameter_file',
        default_value=os.path.join(
            share_dir, 'config', 'world_map.yaml'),
        description='FPath to the ROS2 parameters file to use.')
    
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    # map_server_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("robot_navigation"),
    #             "launch",
    #             "robot_map_server.launch.py",
    #         )
    #     )
    # )

    map_to_odom_node =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['50', '50', '0', '0', '0', '0', '1', 'map', 'odom']
        )
    odom_to_base_link =  Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )
    
    lio_sam_nodes = GroupAction(
        actions=[
            Node(
                package='lio_sam',
                executable='lio_sam_imuPreintegration',
                name='lio_sam_imuPreintegration',
                parameters=[parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_imageProjection',
                name='lio_sam_imageProjection',
                parameters=[parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_featureExtraction',
                name='lio_sam_featureExtraction',
                parameters=[parameter_file],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_mapOptimization',
                name='lio_sam_mapOptimization',
                parameters=[parameter_file],
                output='screen'
            ),
        ]
    )
    navigation_node =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': navigation_param,
            }.items()
        )

    delayed_lio_sam_server =   RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=map_to_odom_node,
            on_start=[lio_sam_nodes],
        )
    )


    return LaunchDescription([
        params_declare,
        sim_declare,
        map_to_odom_node,
        delayed_lio_sam_server,
        navigation_node,
        odom_to_base_link,
        # delayed_lio_sam_server,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map']
        ),



        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen'
        # ),
    ])
