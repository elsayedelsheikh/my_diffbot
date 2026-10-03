cmake_minimum_required(VERSION 3.8)
project(merlin_camera_streamer)

find_package(ament_cmake REQUIRED)

# Runs on the Jetson host via install.sh; installed here so the config ships with the workspace.
install(DIRECTORY
  config systemd
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS install.sh DESTINATION share/${PROJECT_NAME})

ament_package()
