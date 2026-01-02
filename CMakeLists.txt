cmake_minimum_required(VERSION 3.8)
project(self_balancing_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(angles REQUIRED)

find_package(Matplot++ REQUIRED)

add_executable(robot_control_pid src/control_files/robot_control_pid.cpp)
target_link_libraries(robot_control_pid Matplot++::matplot)

add_executable(robot_control_rl 
  src/control_files/robot_control_rl.cpp
  src/control_files/utils/robot_utils.cpp)

install(DIRECTORY src/launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY src/models
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  robot_control_pid
  robot_control_rl
  DESTINATION lib/${PROJECT_NAME})

include_directories(
  include
  /home/rob/ros2_ws/src/third_party
)

ament_target_dependencies(robot_control_pid
  rclcpp
  std_msgs
  sensor_msgs
  ros_gz_interfaces
  angles)

ament_target_dependencies(robot_control_rl
  rclcpp
  std_msgs
  sensor_msgs
  ros_gz_interfaces
  angles)

ament_package()