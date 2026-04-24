cmake_minimum_required(VERSION 3.5)
project(meca500_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Crop pointcloud node
add_executable(crop_pointcloud src/crop_pointcloud.cpp)
ament_target_dependencies(crop_pointcloud rclcpp sensor_msgs geometry_msgs tf2_ros)

install(TARGETS crop_pointcloud DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
    DIRECTORY launch config meshes
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/startup_aruco_calibrator.py
        scripts/publish_table_scene.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
