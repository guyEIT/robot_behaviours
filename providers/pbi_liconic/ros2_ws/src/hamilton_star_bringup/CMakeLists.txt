cmake_minimum_required(VERSION 3.8)
project(hamilton_star_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config systemd
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
