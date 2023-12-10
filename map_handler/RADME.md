README.md
Map Handler

This directory contains the launch file load_map.launch.py for loading a map in a ROS2 environment. The launch file is responsible for setting up and launching three nodes: map_position_change_node, map_server, and lifecycle_manager_map_server.
Dependencies

- ROS2
- ament_index_python
- launch
- launch_ros
- nav2_map_server
- nav2_lifecycle_manager
- map_handler package
Usage

To use the launch file, you need to specify the name of the map file (without the .yaml extension) as a launch argument. You can also specify whether to use simulation time or not.
true
Nodes

- map_position_change_node: This node is part of the map_handler package. It is responsible for handling changes in the map's position. The map file path is passed as a parameter to this node.

- map_server: This node is part of the nav2_map_server package. It is responsible for serving the map. The map configuration, topic name, frame id, and yaml filename are passed as parameters to this node.

- lifecycle_manager_map_server: This node is part of the nav2_lifecycle_manager package. It is responsible for managing the lifecycle of the map_server node. The use of simulation time, autostart, bond timeout, and node names are passed as parameters to this node.
Launch Arguments

- map_name: The name of the map file (without the .yaml extension). Default value is aksk.

- use_sim_time: Whether to use simulation time or not. Default value is true.
Note

The map files should be located in the maps directory of the map_handler package. The map configuration file should be located in the param directory of the robot_navigation package.