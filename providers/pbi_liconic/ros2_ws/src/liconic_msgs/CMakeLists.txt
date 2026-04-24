cmake_minimum_required(VERSION 3.8)
project(liconic_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(action_files
  action/TakeIn.action
  action/Fetch.action
)

set(srv_files
  srv/SetTemperature.srv
  srv/SetHumidity.srv
  srv/GetStatus.srv
  srv/ClearLoadingTray.srv
  srv/ClearCassettePosition.srv
  srv/StartShaking.srv
  srv/StopShaking.srv
  srv/ResetError.srv
)

set(msg_files
  msg/PlateLocation.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
