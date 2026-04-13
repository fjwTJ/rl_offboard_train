cmake_minimum_required(VERSION 3.8)
project(target_uav_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(target_offboard_control src/target_offboard_control.cpp)
ament_target_dependencies(target_offboard_control
  geometry_msgs
  px4_msgs
  rclcpp
  std_msgs
)

add_executable(target_random_motion_node src/target_random_motion_node.cpp)
ament_target_dependencies(target_random_motion_node
  geometry_msgs
  px4_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  target_offboard_control
  target_random_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
