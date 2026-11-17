cmake_minimum_required(VERSION 3.8)
project(ur3e_printhead_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# Use the same geometry profile as the workstation instead of duplicating its
# mesh, mount, TCP candidate and envelope in the offline URDF.
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/printhead_geometry.json
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
