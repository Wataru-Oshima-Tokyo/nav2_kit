cmake_minimum_required(VERSION 3.8)
project(slope_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)
set(dependencies
    rclcpp
    sensor_msgs
    tf2 
    tf2_ros 
    geometry_msgs
    tf2_geometry_msgs
)

add_executable(slope_detection src/slope_detection.cpp)
ament_target_dependencies(slope_detection ${dependencies} )
# rosidl_target_interfaces(slope_detection ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
slope_detection
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
