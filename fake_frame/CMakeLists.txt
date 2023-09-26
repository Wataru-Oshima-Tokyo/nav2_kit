cmake_minimum_required(VERSION 3.8)
project(fake_frame)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(fake_odom_broadcaster src/fake_odom_broadcaster.cpp)
ament_target_dependencies(fake_odom_broadcaster rclcpp tf2 tf2_ros geometry_msgs)

add_executable(fake_point_clouds src/fake_point_clouds.cpp)
ament_target_dependencies(fake_point_clouds rclcpp sensor_msgs)

add_executable(fake_dynamic_tf_broadcaster src/fake_dynamic_tf_broadcaster.cpp)
ament_target_dependencies(fake_dynamic_tf_broadcaster rclcpp tf2_ros geometry_msgs)



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

install(TARGETS
  fake_odom_broadcaster
  fake_point_clouds
  fake_dynamic_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
