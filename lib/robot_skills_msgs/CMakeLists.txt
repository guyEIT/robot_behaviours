cmake_minimum_required(VERSION 3.8)
project(robot_skills_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/DetectedObject.msg"
  "msg/KeyValue.msg"
  "msg/LogEvent.msg"
  "msg/SkillDescription.msg"
  "msg/SkillAdvertisement.msg"
  "msg/SkillManifest.msg"
  "msg/TaskState.msg"
  "msg/TaskStep.msg"
  "msg/HumanPrompt.msg"
  "msg/HumanResponse.msg"
  "msg/LeaseState.msg"
  "msg/LeaseEvent.msg"
  "msg/DryRunStatus.msg"
  # Services
  "srv/GetSkillDescriptions.srv"
  "srv/ComposeTask.srv"
  "srv/RegisterCompoundSkill.srv"
  "srv/RegisterScript.srv"
  "srv/AcquireLease.srv"
  "srv/RenewLease.srv"
  "srv/ReleaseLease.srv"
  "srv/ApproveDryRun.srv"
  # Long-lived campaign operator services (week-2)
  "srv/AddPlate.srv"
  "srv/RetirePlate.srv"
  "srv/PauseCampaign.srv"
  "srv/OperatorDecision.srv"
  "srv/CancelActiveTask.srv"
  "srv/ValidatePlan.srv"
  # Actions
  "action/MoveToCartesianPose.action"
  "action/MoveToNamedConfig.action"
  "action/MoveToJointConfig.action"
  "action/MoveCartesianLinear.action"
  "action/GripperControl.action"
  "action/FrankaGripperControl.action"
  "action/DetectObject.action"
  "action/CapturePointCloud.action"
  "action/SetDigitalIO.action"
  "action/CheckCollision.action"
  "action/UpdatePlanningScene.action"
  "action/RobotEnable.action"
  "action/RecordRosbag.action"
  "action/StopRecording.action"
  "action/CheckSystemReady.action"
  "action/ExecuteBehaviorTree.action"
  "action/RunScript.action"
  "action/ImagePlate.action"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    std_msgs
    action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
