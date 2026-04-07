cmake_minimum_required(VERSION 3.8)
project(robot_mock_skill_atoms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_skills_msgs REQUIRED)
# skill_base.hpp is copied locally to avoid MoveIt2 dependency from robot_skill_atoms
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(MOCK_DEPS
  rclcpp
  rclcpp_action
  rclcpp_components
  robot_skills_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  diagnostic_updater
  diagnostic_msgs
)

# ── Mock MoveToNamedConfig skill ─────────────────────────────────────────────
add_library(mock_move_to_named_config SHARED
  src/mock_move_to_named_config.cpp)
target_include_directories(mock_move_to_named_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_move_to_named_config ${MOCK_DEPS})
rclcpp_components_register_node(mock_move_to_named_config
  PLUGIN "robot_mock_skill_atoms::MockMoveToNamedConfig"
  EXECUTABLE mock_move_to_named_config_node)

# ── Mock MoveToCartesianPose skill ───────────────────────────────────────────
add_library(mock_move_to_cartesian_pose SHARED
  src/mock_move_to_cartesian_pose.cpp)
target_include_directories(mock_move_to_cartesian_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_move_to_cartesian_pose ${MOCK_DEPS})
rclcpp_components_register_node(mock_move_to_cartesian_pose
  PLUGIN "robot_mock_skill_atoms::MockMoveToCartesianPose"
  EXECUTABLE mock_move_to_cartesian_pose_node)

# ── Mock GripperControl skill ────────────────────────────────────────────────
add_library(mock_gripper_control SHARED
  src/mock_gripper_control.cpp)
target_include_directories(mock_gripper_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_gripper_control ${MOCK_DEPS})
rclcpp_components_register_node(mock_gripper_control
  PLUGIN "robot_mock_skill_atoms::MockGripperControl"
  EXECUTABLE mock_gripper_control_node)

# ── Mock DetectObject skill ──────────────────────────────────────────────────
add_library(mock_detect_object SHARED
  src/mock_detect_object.cpp)
target_include_directories(mock_detect_object PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_detect_object ${MOCK_DEPS})
rclcpp_components_register_node(mock_detect_object
  PLUGIN "robot_mock_skill_atoms::MockDetectObject"
  EXECUTABLE mock_detect_object_node)

# ── Mock MoveToJointConfig skill ─────────────────────────────────────────────
add_library(mock_move_to_joint_config SHARED src/mock_move_to_joint_config.cpp)
target_include_directories(mock_move_to_joint_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_move_to_joint_config ${MOCK_DEPS})
rclcpp_components_register_node(mock_move_to_joint_config
  PLUGIN "robot_mock_skill_atoms::MockMoveToJointConfig"
  EXECUTABLE mock_move_to_joint_config_node)

# ── Mock MoveCartesianLinear skill ───────────────────────────────────────────
add_library(mock_move_cartesian_linear SHARED src/mock_move_cartesian_linear.cpp)
target_include_directories(mock_move_cartesian_linear PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_move_cartesian_linear ${MOCK_DEPS})
rclcpp_components_register_node(mock_move_cartesian_linear
  PLUGIN "robot_mock_skill_atoms::MockMoveCartesianLinear"
  EXECUTABLE mock_move_cartesian_linear_node)

# ── Mock CapturePointCloud skill ─────────────────────────────────────────────
add_library(mock_capture_point_cloud SHARED src/mock_capture_point_cloud.cpp)
target_include_directories(mock_capture_point_cloud PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_capture_point_cloud ${MOCK_DEPS})
rclcpp_components_register_node(mock_capture_point_cloud
  PLUGIN "robot_mock_skill_atoms::MockCapturePointCloud"
  EXECUTABLE mock_capture_point_cloud_node)

# ── Mock SetDigitalIO skill ──────────────────────────────────────────────────
add_library(mock_set_digital_io SHARED src/mock_set_digital_io.cpp)
target_include_directories(mock_set_digital_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_set_digital_io ${MOCK_DEPS})
rclcpp_components_register_node(mock_set_digital_io
  PLUGIN "robot_mock_skill_atoms::MockSetDigitalIO"
  EXECUTABLE mock_set_digital_io_node)

# ── Mock CheckCollision skill ────────────────────────────────────────────────
add_library(mock_check_collision SHARED src/mock_check_collision.cpp)
target_include_directories(mock_check_collision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_check_collision ${MOCK_DEPS})
rclcpp_components_register_node(mock_check_collision
  PLUGIN "robot_mock_skill_atoms::MockCheckCollision"
  EXECUTABLE mock_check_collision_node)

# ── Mock UpdatePlanningScene skill ───────────────────────────────────────────
add_library(mock_update_planning_scene SHARED src/mock_update_planning_scene.cpp)
target_include_directories(mock_update_planning_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_update_planning_scene ${MOCK_DEPS})
rclcpp_components_register_node(mock_update_planning_scene
  PLUGIN "robot_mock_skill_atoms::MockUpdatePlanningScene"
  EXECUTABLE mock_update_planning_scene_node)

# ── Mock RobotEnable skill ───────────────────────────────────────────────────
add_library(mock_robot_enable SHARED src/mock_robot_enable.cpp)
target_include_directories(mock_robot_enable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_robot_enable ${MOCK_DEPS})
rclcpp_components_register_node(mock_robot_enable
  PLUGIN "robot_mock_skill_atoms::MockRobotEnable"
  EXECUTABLE mock_robot_enable_node)

# ── Mock RecordRosbag skill ──────────────────────────────────────────────────
add_library(mock_record_rosbag SHARED src/mock_record_rosbag.cpp)
target_include_directories(mock_record_rosbag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_record_rosbag ${MOCK_DEPS})
rclcpp_components_register_node(mock_record_rosbag
  PLUGIN "robot_mock_skill_atoms::MockRecordRosbag"
  EXECUTABLE mock_record_rosbag_node)

# ── Mock CheckSystemReady skill ──────────────────────────────────────────────
add_library(mock_check_system_ready SHARED src/mock_check_system_ready.cpp)
target_include_directories(mock_check_system_ready PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_check_system_ready ${MOCK_DEPS})
rclcpp_components_register_node(mock_check_system_ready
  PLUGIN "robot_mock_skill_atoms::MockCheckSystemReady"
  EXECUTABLE mock_check_system_ready_node)

# ── Mock Joint State Publisher ───────────────────────────────────────────────
add_library(mock_joint_state_publisher SHARED
  src/mock_joint_state_publisher.cpp)
target_include_directories(mock_joint_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_joint_state_publisher ${MOCK_DEPS})
rclcpp_components_register_node(mock_joint_state_publisher
  PLUGIN "robot_mock_skill_atoms::MockJointStatePublisher"
  EXECUTABLE mock_joint_state_publisher_node)

# ── Main component manager node ──────────────────────────────────────────────
add_executable(mock_skills_node src/mock_skills_node.cpp)
ament_target_dependencies(mock_skills_node rclcpp rclcpp_components)
target_link_libraries(mock_skills_node
  rclcpp_components::component_manager
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS
  mock_move_to_named_config
  mock_move_to_cartesian_pose
  mock_move_to_joint_config
  mock_move_cartesian_linear
  mock_gripper_control
  mock_detect_object
  mock_capture_point_cloud
  mock_set_digital_io
  mock_check_collision
  mock_update_planning_scene
  mock_robot_enable
  mock_record_rosbag
  mock_check_system_ready
  mock_joint_state_publisher
  mock_skills_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  mock_move_to_named_config
  mock_move_to_cartesian_pose
  mock_move_to_joint_config
  mock_move_cartesian_linear
  mock_gripper_control
  mock_detect_object
  mock_capture_point_cloud
  mock_set_digital_io
  mock_check_collision
  mock_update_planning_scene
  mock_robot_enable
  mock_record_rosbag
  mock_check_system_ready
  mock_joint_state_publisher
)
ament_export_dependencies(${MOCK_DEPS})
ament_package()
