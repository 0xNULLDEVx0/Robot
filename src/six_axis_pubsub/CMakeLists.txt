cmake_minimum_required(VERSION 3.8)
project(six_axis_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executables
add_executable(arm_controller src/arm_controller.cpp)
ament_target_dependencies(arm_controller rclcpp trajectory_msgs sensor_msgs)
install(TARGETS arm_controller DESTINATION lib/${PROJECT_NAME})

add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp trajectory_msgs)
install(TARGETS teleop_node DESTINATION lib/${PROJECT_NAME})

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
