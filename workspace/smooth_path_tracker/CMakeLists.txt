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

include_directories(
  include
)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  visualization_msgs
)

# Add path processor library
add_library(path_processor_lib SHARED src/path_processor.cpp)
ament_target_dependencies(path_processor_lib
  ${dependencies}
)

# Add path smoother node
add_executable(path_smoother_node src/path_smoother_node.cpp src/path_processor.cpp)
ament_target_dependencies(path_smoother_node
  ${dependencies}
)

# Add path tracker node
add_executable(path_tracker_node src/path_tracker_node.cpp)
add_library(path_tracker_node_lib SHARED src/path_tracker_node.cpp)
ament_target_dependencies(path_tracker_node
  ${dependencies}
)
ament_target_dependencies(path_tracker_node_lib
  ${dependencies}
)

install(TARGETS
  path_processor_lib
  path_tracker_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

# Install launch and config files
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()