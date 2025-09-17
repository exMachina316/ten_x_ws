cmake_minimum_required(VERSION 3.8)
project(smooth_path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add path smoother node
add_executable(path_smoother_node src/path_smoother_node.cpp src/path_processor.cpp)
ament_target_dependencies(path_smoother_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)
target_include_directories(path_smoother_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add path tracker node
add_executable(path_tracker_node src/path_tracker_node.cpp)
ament_target_dependencies(path_tracker_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  path_smoother_node
  path_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()