cmake_minimum_required(VERSION 3.8)
project(robot_skill_atoms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_skills_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)

set(SKILL_DEPS
  rclcpp
  rclcpp_action
  rclcpp_components
  robot_skills_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  control_msgs
)

# ── MoveToNamedConfig skill ──────────────────────────────────────────────────
add_library(move_to_named_config_skill SHARED
  src/move_to_named_config_skill.cpp)
target_include_directories(move_to_named_config_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(move_to_named_config_skill ${SKILL_DEPS})
rclcpp_components_register_node(move_to_named_config_skill
  PLUGIN "robot_skill_atoms::MoveToNamedConfigSkill"
  EXECUTABLE move_to_named_config_skill_node)

# ── MoveToCartesianPose skill ────────────────────────────────────────────────
add_library(move_to_cartesian_pose_skill SHARED
  src/move_to_cartesian_pose_skill.cpp)
target_include_directories(move_to_cartesian_pose_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(move_to_cartesian_pose_skill ${SKILL_DEPS})
rclcpp_components_register_node(move_to_cartesian_pose_skill
  PLUGIN "robot_skill_atoms::MoveToCartesianPoseSkill"
  EXECUTABLE move_to_cartesian_pose_skill_node)

# ── GripperControl skill ─────────────────────────────────────────────────────
add_library(gripper_control_skill SHARED
  src/gripper_control_skill.cpp)
target_include_directories(gripper_control_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gripper_control_skill ${SKILL_DEPS})
rclcpp_components_register_node(gripper_control_skill
  PLUGIN "robot_skill_atoms::GripperControlSkill"
  EXECUTABLE gripper_control_skill_node)

# ── DetectObject skill ───────────────────────────────────────────────────────
add_library(detect_object_skill SHARED
  src/detect_object_skill.cpp)
target_include_directories(detect_object_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(detect_object_skill ${SKILL_DEPS})
rclcpp_components_register_node(detect_object_skill
  PLUGIN "robot_skill_atoms::DetectObjectSkill"
  EXECUTABLE detect_object_skill_node)

# ── Main component manager node ──────────────────────────────────────────────
add_executable(skills_node src/skills_node.cpp)
ament_target_dependencies(skills_node rclcpp rclcpp_components)
target_link_libraries(skills_node)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS
  move_to_named_config_skill
  move_to_cartesian_pose_skill
  gripper_control_skill
  detect_object_skill
  skills_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  move_to_named_config_skill
  move_to_cartesian_pose_skill
  gripper_control_skill
  detect_object_skill
)
ament_export_dependencies(${SKILL_DEPS})
ament_package()
