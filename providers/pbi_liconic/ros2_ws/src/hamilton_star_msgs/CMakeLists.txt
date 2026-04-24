cmake_minimum_required(VERSION 3.8)
project(hamilton_star_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(action_files
  action/PickUpTips.action
  action/DropTips.action
  action/Aspirate.action
  action/Dispense.action
  action/Transfer.action
  action/PickUpTips96.action
  action/DropTips96.action
  action/Aspirate96.action
  action/Dispense96.action
  action/MoveResource.action
  action/PickUpCoreGripper.action
  action/ReturnCoreGripper.action
  action/JogChannel.action
  action/HandoffTransfer.action
)

set(srv_files
  srv/InitializeModule.srv
  srv/LoadDeck.srv
  srv/SerializeDeck.srv
  srv/GetStatus.srv
  srv/ListResources.srv
  srv/ResetError.srv
  srv/DefineHandoff.srv
  srv/ListHandoffs.srv
  srv/DeleteHandoff.srv
)

set(msg_files
  msg/Status.msg
  msg/Event.msg
  msg/ChannelState.msg
  msg/Handoff.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs std_srvs geometry_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
