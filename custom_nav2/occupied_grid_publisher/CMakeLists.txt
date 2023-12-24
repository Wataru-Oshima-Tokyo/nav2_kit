cmake_minimum_required(VERSION 3.8)
project(occupied_grid_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(techshare_ros_pkg2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)



add_executable(occupied_grid_publisher src/occupied_grid_publisher.cpp)
add_executable(safety_costmap src/safety_costmap.cpp)
add_executable(sample_random_costmap src/sample_random_costmap.cpp)

# It's important to place this after the custom message generation

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  techshare_ros_pkg2
  std_srvs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  nav2_costmap_2d
  nav2_msgs
)

ament_target_dependencies(occupied_grid_publisher ${dependencies})
ament_target_dependencies(safety_costmap ${dependencies})
ament_target_dependencies(sample_random_costmap ${dependencies})

# You should depend on the message generation before building the executable

install(TARGETS
  occupied_grid_publisher
  safety_costmap
  sample_random_costmap
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
