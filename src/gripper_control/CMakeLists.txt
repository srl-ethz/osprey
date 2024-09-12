cmake_minimum_required(VERSION 3.13)

project(gripper_control)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ros custom
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
# gripper control
add_executable(gripper_control src/gripper_control_app.cpp)
target_include_directories(gripper_control PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(gripper_control PUBLIC 
  src/gripper_control.cpp
  src/serialib.cpp)
ament_target_dependencies(gripper_control
  rclcpp 
  osprey_interface)
# target_include_directories(gripper_control PUBLIC include)


# INSTALLATION
install(TARGETS
  gripper_control
  DESTINATION lib/${PROJECT_NAME})


ament_package()
