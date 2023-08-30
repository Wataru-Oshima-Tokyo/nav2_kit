import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    default_nav_to_pose_bt_xml_robot = os.path.join(get_package_share_directory(
    'robot_navigation'), 'config', 'behavior_robot.xml')
    controller_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', 'controller_robot.yaml')
    bt_navigator_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', 'bt_navigator_robot.yaml')
    planner_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', 'planner_server_robot.yaml')
    recovery_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', 'recovery_robot.yaml')

    waypoints_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', 'waypoint_follower_robot.yaml')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    return LaunchDescription([

        # Nodes for robot

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_robot],
            remappings=remappings
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_robot],
            remappings=remappings
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_yaml_robot],
            remappings=remappings
        ),

        Node(
            namespace='',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{bt_navigator_yaml_robot},{'default_bt_xml_filename': default_nav_to_pose_bt_xml_robot}],
            remappings=remappings
            # parameters=[bt_navigator_yaml_robot]
            ),

        Node(
            namespace='',
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoints_yaml_robot],
            remappings=remappings
            ),        


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'use_sim_time': True},
                        {'node_names': [
                            'controller_server',
                            'planner_server',
                            'recoveries_server',
                            'bt_navigator',
                            'waypoint_follower'
                        ]}]),

        # Node(
        #     namespace='',
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', robot_rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
