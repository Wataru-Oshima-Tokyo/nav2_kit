cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(cmdvel_to_odom src/cmdvel_to_odom.cpp)
ament_target_dependencies(cmdvel_to_odom rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros )
add_executable(safety_relay src/safety_relay.cpp)
ament_target_dependencies(safety_relay rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS cmdvel_to_odom safety_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  src
  param
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
