import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import distro

def get_ros2_distro():
    ubuntu_version = distro.version()
    if ubuntu_version == '20.04':
        return 'foxy'
    elif ubuntu_version == '22.04':
        return 'humble'
    else:
        return 'unknown'


def launch_setup(context, *args, **kwargs):
    ros2_distro = get_ros2_distro()
    print('ROS 2 Distro:', ros2_distro)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name').perform(context)

    default_nav_to_pose_bt_xml_robot = os.path.join(get_package_share_directory(
    'robot_navigation'), 'config', robot_name, ros2_distro, 'nav_thr_poses.xml')
    controller_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', robot_name, ros2_distro, 'controller_robot.yaml')
    bt_navigator_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', robot_name, ros2_distro, 'bt_navigator_robot.yaml')
    planner_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', robot_name, ros2_distro, 'planner_server_robot.yaml')
    recovery_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', robot_name, ros2_distro, 'recovery_robot.yaml')

    waypoints_yaml_robot = os.path.join(get_package_share_directory(
        'robot_navigation'), 'config', 'waypoint_follower_robot.yaml')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/cmd_vel', 'nav_cmd_vel')]

    if (ros2_distro=="humble"):
        behavior_node =  Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml_robot],
            output='screen',
            remappings=remappings
            )
    else:
        behavior_node =  Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_yaml_robot],
            remappings=remappings
        )


    return [
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


    ]
def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='limo_ackermann', description='Robot Name')



    return LaunchDescription([
        robot_name_arg,
        OpaqueFunction(function=launch_setup)

    ])
