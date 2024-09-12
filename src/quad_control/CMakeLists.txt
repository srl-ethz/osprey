cmake_minimum_required(VERSION 3.13)

project(quad_control)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# ros custom
find_package(osprey_interface REQUIRED)
# external
find_package(MAVSDK REQUIRED)


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
# quad control
add_executable(quad_control src/apps/quad_control.cpp)
target_include_directories(quad_control PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(quad_control PUBLIC 
  src/quad/quad.cpp
  src/actions/takeoff.cpp
  src/actions/goToPos.cpp
  src/actions/hover_acc.cpp
  src/wrappers/mavsdk_wrapper.cpp)
ament_target_dependencies(quad_control 
  rclcpp 
  rclcpp_action
  geometry_msgs
  osprey_interface)
target_link_libraries(quad_control MAVSDK::mavsdk)

# pose transformer
add_executable(pose_transformer src/apps/pose_transformer.cpp)
target_include_directories(pose_transformer PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(pose_transformer PUBLIC)
ament_target_dependencies(pose_transformer
  rclcpp
  tf2
  geometry_msgs
  nav_msgs
  osprey_interface)


# INSTALLATION
install(TARGETS
  quad_control
  pose_transformer
  DESTINATION lib/${PROJECT_NAME})


ament_package()