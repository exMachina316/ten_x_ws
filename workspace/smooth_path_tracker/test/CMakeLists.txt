# Tests for Path Tracker Node
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_path_tracker_node
  test_path_tracker_node.cpp
)
ament_target_dependencies(test_path_tracker_node
  ${dependencies}
)

ament_add_gtest(test_path_processor
  test_path_processor.cpp
)

# Ensure the test executable is linked against the path_tracker_node library
target_link_libraries(test_path_tracker_node
  path_tracker_node_lib
)

# Ensure the test executable is linked against the path_processor library
target_link_libraries(test_path_processor
  path_processor_lib
)

ament_target_dependencies(test_path_processor
  ${dependencies}
)
