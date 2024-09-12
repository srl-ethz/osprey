cmake_minimum_required(VERSION 3.13)

project(mission_control)


# DEPENDENCIES
# ros
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(osprey_interface REQUIRED)


# Compiler options
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# Compile flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# TARGETS
set(sourcfiles
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/takeoff.cpp
  src/actions/land.cpp
  src/actions/go_to_pos.cpp
  src/actions/set_gripper.cpp
  # src/actions/hover_acc.cpp
)
set(dependencies
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface
)

# takeoff_test
add_executable(takeoff_test src/missions/takeoff_test.cpp)
target_include_directories(takeoff_test PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(takeoff_test PUBLIC ${sourcfiles})
ament_target_dependencies(takeoff_test ${dependencies})

# square
add_executable(square src/missions/square.cpp)
target_include_directories(square PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(square PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp)
ament_target_dependencies(square 
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface)

# gripper_test_static
add_executable(gripper_test_static src/missions/gripper_test_static.cpp)
target_include_directories(gripper_test_static PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(gripper_test_static PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp
  src/actions/set_gripper.cpp)
ament_target_dependencies(gripper_test_static 
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface)

# gripper_test_setpoint
add_executable(gripper_test_setpoint src/missions/gripper_test_setpoint.cpp)
target_include_directories(gripper_test_setpoint PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(gripper_test_setpoint PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp
  src/actions/set_gripper.cpp)
ament_target_dependencies(gripper_test_setpoint 
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface)

# go_to_object_test
add_executable(go_to_object_test src/missions/go_to_object_test.cpp)
target_include_directories(go_to_object_test PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(go_to_object_test PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp
  src/actions/set_gripper.cpp)
ament_target_dependencies(go_to_object_test 
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface)

# grasp
add_executable(grasp src/missions/grasp.cpp)
target_include_directories(grasp PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(grasp PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp
  src/actions/set_gripper.cpp)
ament_target_dependencies(grasp 
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface)

# swoop
add_executable(swoop src/missions/swoop.cpp)
target_include_directories(swoop PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(swoop PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp
  src/actions/set_gripper.cpp)
ament_target_dependencies(swoop 
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface)

# object_search
add_executable(object_search src/missions/object_search.cpp)
target_include_directories(object_search PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(object_search PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp
  src/actions/set_gripper.cpp)
ament_target_dependencies(object_search 
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface)


# INSTALLATION
install(TARGETS
  takeoff_test
  square
  swoop
  grasp
  gripper_test_setpoint
  gripper_test_static
  go_to_object_test
  object_search
  DESTINATION lib/${PROJECT_NAME})


ament_package()