cmake_minimum_required(VERSION 3.8)
project(robot_arm_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_skills_msgs REQUIRED)
find_package(robot_skill_advertise REQUIRED)
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
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(SKILL_DEPS
  rclcpp
  rclcpp_action
  rclcpp_components
  robot_skills_msgs
  robot_skill_advertise
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
  controller_manager_msgs
  diagnostic_updater
  diagnostic_msgs
)

# ── MoveToNamedConfig skill ──────────────────────────────────────────────────
add_library(move_to_named_config_skill SHARED
  src/move_to_named_config_skill.cpp)
target_include_directories(move_to_named_config_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(move_to_named_config_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(move_to_named_config_skill "robot_arm_skills::MoveToNamedConfigSkill")

# ── MoveToCartesianPose skill ────────────────────────────────────────────────
add_library(move_to_cartesian_pose_skill SHARED
  src/move_to_cartesian_pose_skill.cpp)
target_include_directories(move_to_cartesian_pose_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(move_to_cartesian_pose_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(move_to_cartesian_pose_skill "robot_arm_skills::MoveToCartesianPoseSkill")

# ── GripperControl skill ─────────────────────────────────────────────────────
add_library(gripper_control_skill SHARED
  src/gripper_control_skill.cpp)
target_include_directories(gripper_control_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gripper_control_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(gripper_control_skill "robot_arm_skills::GripperControlSkill")

# ── DetectObject skill ───────────────────────────────────────────────────────
add_library(detect_object_skill SHARED
  src/detect_object_skill.cpp)
target_include_directories(detect_object_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(detect_object_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(detect_object_skill "robot_arm_skills::DetectObjectSkill")

# ── CheckSystemReady skill ───────────────────────────────────────────────────
add_library(check_system_ready_skill SHARED
  src/check_system_ready_skill.cpp)
target_include_directories(check_system_ready_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(check_system_ready_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(check_system_ready_skill "robot_arm_skills::CheckSystemReadySkill")

# ── MoveToJointConfig skill ──────────────────────────────────────────────────
add_library(move_to_joint_config_skill SHARED
  src/move_to_joint_config_skill.cpp)
target_include_directories(move_to_joint_config_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(move_to_joint_config_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(move_to_joint_config_skill "robot_arm_skills::MoveToJointConfigSkill")

# ── MoveCartesianLinear skill ────────────────────────────────────────────────
add_library(move_cartesian_linear_skill SHARED
  src/move_cartesian_linear_skill.cpp)
target_include_directories(move_cartesian_linear_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(move_cartesian_linear_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(move_cartesian_linear_skill "robot_arm_skills::MoveCartesianLinearSkill")

# ── UpdatePlanningScene skill ────────────────────────────────────────────────
add_library(update_planning_scene_skill SHARED
  src/update_planning_scene_skill.cpp)
target_include_directories(update_planning_scene_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(update_planning_scene_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(update_planning_scene_skill "robot_arm_skills::UpdatePlanningSceneSkill")

# ── CheckCollision skill ────────────────────────────────────────────────────
add_library(check_collision_skill SHARED
  src/check_collision_skill.cpp)
target_include_directories(check_collision_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(check_collision_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(check_collision_skill "robot_arm_skills::CheckCollisionSkill")

# ── RobotEnable skill ───────────────────────────────────────────────────────
add_library(robot_enable_skill SHARED
  src/robot_enable_skill.cpp)
target_include_directories(robot_enable_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_enable_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(robot_enable_skill "robot_arm_skills::RobotEnableSkill")

# ── CapturePointCloud skill ─────────────────────────────────────────────────
add_library(capture_point_cloud_skill SHARED
  src/capture_point_cloud_skill.cpp)
target_include_directories(capture_point_cloud_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(capture_point_cloud_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(capture_point_cloud_skill "robot_arm_skills::CapturePointCloudSkill")

# ── SetDigitalIO skill ──────────────────────────────────────────────────────
add_library(set_digital_io_skill SHARED
  src/set_digital_io_skill.cpp)
target_include_directories(set_digital_io_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(set_digital_io_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(set_digital_io_skill "robot_arm_skills::SetDigitalIOSkill")

# ── RecordRosbag skill ──────────────────────────────────────────────────────
add_library(record_rosbag_skill SHARED
  src/record_rosbag_skill.cpp)
target_include_directories(record_rosbag_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(record_rosbag_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(record_rosbag_skill "robot_arm_skills::RecordRosbagSkill")

# ── StopRecording skill ─────────────────────────────────────────────────────
add_library(stop_recording_skill SHARED
  src/stop_recording_skill.cpp)
target_include_directories(stop_recording_skill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(stop_recording_skill ${SKILL_DEPS})
rclcpp_components_register_nodes(stop_recording_skill "robot_arm_skills::StopRecordingSkill")

# ── Install ──────────────────────────────────────────────────────────────────
# Per-skill libraries are loaded as components by per-robot proxies (e.g.
# meca500_skill_server) via rclcpp_components::component_container_mt. The
# old standalone *_skill_node executables and skills_node component-manager
# wrapper are gone — phase 4 collapsed the fleet into one process.
install(TARGETS
  move_to_named_config_skill
  move_to_cartesian_pose_skill
  gripper_control_skill
  detect_object_skill
  check_system_ready_skill
  move_to_joint_config_skill
  move_cartesian_linear_skill
  update_planning_scene_skill
  check_collision_skill
  robot_enable_skill
  capture_point_cloud_skill
  set_digital_io_skill
  record_rosbag_skill
  stop_recording_skill
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
  check_system_ready_skill
  move_to_joint_config_skill
  move_cartesian_linear_skill
  update_planning_scene_skill
  check_collision_skill
  robot_enable_skill
  capture_point_cloud_skill
  set_digital_io_skill
  record_rosbag_skill
  stop_recording_skill
)
ament_export_dependencies(${SKILL_DEPS})
ament_package()
