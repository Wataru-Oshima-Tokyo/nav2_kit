
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math
import os
from ament_index_python.packages import get_package_share_directory
    
def generate_launch_description():
    # Declare the launch arguments
    min_height_for_move_ = LaunchConfiguration('min_height_for_move')
    min_height_for_move_arg = DeclareLaunchArgument(
        'min_height_for_move',
        default_value="-0.1",
        description='Top-level namespace')


    # sim_ = LaunchConfiguration('use_sim_time')
    # use_sim_time_arg = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value="False",
    #     description='use_sim_time')
    # if sim_ == "true" or sim_ ==  "True":
    #     use_sim_time_ = True
    # else:
    #     use_sim_time_ = False
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Whether to use simulation time or not'
    )
    theta_min_deg = -60
    theta_max_deg = 60

    theta_min_rad = theta_min_deg * math.pi / 180
    theta_max_rad = theta_max_deg * math.pi / 180
    occupied_cell_node = Node(
            package='occupied_grid_publisher',
            executable='occupied_grid_publisher',
            name='occupied_grid_publisher',
            output="screen",
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  
    )


    fake_scan_move = Node(
        package='fake_frame',
        executable='fake_scan',
        name='fake_base_link',
            parameters=[{'target_topic': "scan_for_move"}]    
    )

    fake_scan = Node(
        package='fake_frame',
        executable='fake_scan',
        name='fake_base_link',
            parameters=[{'target_topic': "scan"}]    
    )


    return LaunchDescription([
        min_height_for_move_arg,
        use_sim_time_arg,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_to_cloud',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser_link']
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'laser_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 4.0,
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 120.0,
                'use_inf': False,
                'inf_epsilon': 4.0,
                'qos': "reliable",
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan_for_safety')],
            parameters=[{
                'target_frame': 'laser_link',
                'transform_tolerance': 0.01,
                'min_height': -10.0,
                'max_height': 0.0,
                'angle_min': theta_min_rad,
                'angle_max': theta_max_rad,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 120.0,
                'use_inf': True,
                'inf_epsilon': 4.0,
                'qos': "reliable",
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/points_raw'),
                        ('scan', '/scan_for_move')],
            parameters=[{
                'target_frame': 'laser_link',
                'transform_tolerance': 0.01,
                'min_height': min_height_for_move_,
                'max_height': 3.0,
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 7.0,
                'use_inf': False,
                'inf_epsilon': 4.0,
                'qos': "reliable",
                'use_sim_time': LaunchConfiguration('use_sim_time'),

            }],
            name='pointcloud_to_laserscan_for_move'
        ),
        fake_scan_move,
        fake_scan,
        occupied_cell_node,

    ])
