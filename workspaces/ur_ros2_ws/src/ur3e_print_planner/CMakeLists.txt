cmake_minimum_required(VERSION 3.8)
project(ur3e_print_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ur_klipper_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

get_filename_component(PRINTHEAD_GEOMETRY_CONFIG
  "${CMAKE_CURRENT_SOURCE_DIR}/../../config/printhead_geometry.json" ABSOLUTE)
set(PRINTHEAD_GEOMETRY_GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_printhead_geometry.py")
set(PRINTHEAD_GEOMETRY_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/printhead_collision_geometry_data.hpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${PRINTHEAD_GEOMETRY_CONFIG}" "${PRINTHEAD_GEOMETRY_GENERATOR}")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${PRINTHEAD_GEOMETRY_GENERATOR}"
    --config "${PRINTHEAD_GEOMETRY_CONFIG}" --output "${PRINTHEAD_GEOMETRY_HEADER}"
  RESULT_VARIABLE PRINTHEAD_GEOMETRY_RESULT
  ERROR_VARIABLE PRINTHEAD_GEOMETRY_ERROR
)
if(NOT PRINTHEAD_GEOMETRY_RESULT EQUAL 0)
  message(FATAL_ERROR "Cannot generate printhead collision geometry: ${PRINTHEAD_GEOMETRY_ERROR}")
endif()
include_directories("${CMAKE_CURRENT_BINARY_DIR}/generated")

add_executable(plan_box src/plan_box.cpp)
ament_target_dependencies(plan_box
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  shape_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

target_compile_features(plan_box PUBLIC cxx_std_17)

add_executable(cartesian_xyz_moveit src/cartesian_xyz_moveit.cpp)
ament_target_dependencies(cartesian_xyz_moveit
  control_msgs
  controller_manager_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  trajectory_msgs
  ur_klipper_interfaces
)

target_compile_features(cartesian_xyz_moveit PUBLIC cxx_std_17)

add_executable(gcode_path_review src/gcode_path_review.cpp)
ament_target_dependencies(gcode_path_review
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  tf2
  trajectory_msgs
)

target_compile_features(gcode_path_review PUBLIC cxx_std_17)

add_library(joint_sample_validation STATIC src/joint_sample_validation.cpp)
target_include_directories(joint_sample_validation PUBLIC include)
ament_target_dependencies(joint_sample_validation
  geometry_msgs
  moveit_msgs
  shape_msgs
)
target_compile_features(joint_sample_validation PUBLIC cxx_std_17)

add_executable(validate_joint_samples src/validate_joint_samples.cpp)
target_include_directories(validate_joint_samples PRIVATE include)
target_link_libraries(validate_joint_samples joint_sample_validation)
ament_target_dependencies(validate_joint_samples
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
)
target_compile_features(validate_joint_samples PUBLIC cxx_std_17)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_joint_sample_validation test/test_joint_sample_validation.cpp)
  target_include_directories(test_joint_sample_validation PRIVATE include)
  target_link_libraries(test_joint_sample_validation joint_sample_validation)
  ament_target_dependencies(test_joint_sample_validation
    geometry_msgs
    moveit_msgs
    shape_msgs
  )
  target_compile_features(test_joint_sample_validation PUBLIC cxx_std_17)
  add_test(NAME joint_sample_validation COMMAND test_joint_sample_validation)

  add_executable(test_trajectory_safety test/test_trajectory_safety.cpp)
  target_compile_features(test_trajectory_safety PUBLIC cxx_std_17)
  add_test(NAME trajectory_safety COMMAND test_trajectory_safety)
endif()

install(TARGETS plan_box cartesian_xyz_moveit gcode_path_review validate_joint_samples
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
