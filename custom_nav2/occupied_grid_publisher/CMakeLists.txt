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
add_executable(occupied_grid_publisher src/occupied_grid_publisher.cpp)

# It's important to place this after the custom message generation
ament_target_dependencies(occupied_grid_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  techshare_ros_pkg2
  std_srvs
)

# You should depend on the message generation before building the executable

install(TARGETS
  occupied_grid_publisher
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
