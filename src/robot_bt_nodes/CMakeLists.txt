cmake_minimum_required(VERSION 3.8)
project(robot_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(robot_skills_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPS
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  robot_skills_msgs
  geometry_msgs
  std_msgs
)

# ── BT nodes shared library (loaded as plugin by the executor) ────────────────
add_library(robot_bt_nodes SHARED
  src/bt_nodes_plugin.cpp
)
target_include_directories(robot_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_bt_nodes ${DEPS})

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS robot_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install the BT node descriptions for Groot2 palette
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_bt_nodes)
ament_export_dependencies(${DEPS})
ament_package()
