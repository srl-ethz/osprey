cmake_minimum_required(VERSION 3.8)

project(vicon)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



# external
find_package(MAVSDK REQUIRED)


# Compiler options
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# Compile flags
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# TARGETS
# vicon datastream
add_subdirectory(vicon_datastream)

# vicon_px4_interface
add_executable(vicon_px4_interface src/vicon_px4_interface.cpp)
target_include_directories(vicon_px4_interface PUBLIC include)
ament_target_dependencies(vicon_px4_interface 
  rclcpp
  geometry_msgs
  tf2
  tf2_ros)
target_link_libraries(vicon_px4_interface MAVSDK::mavsdk)
  
# vicon_publisher
add_executable(vicon_publisher src/vicon_publisher.cpp)
target_include_directories(vicon_publisher PUBLIC include)
ament_target_dependencies(vicon_publisher 
  rclcpp
  geometry_msgs
  tf2
  tf2_ros)

# vio_px4_interface
add_executable(vio_px4_interface src/vio_px4_interface.cpp)
target_include_directories(vio_px4_interface PUBLIC include)
ament_target_dependencies(vio_px4_interface 
  rclcpp
  geometry_msgs
  tf2
  tf2_ros)
target_link_libraries(vio_px4_interface MAVSDK::mavsdk)

# rovio_px4_interface
add_executable(rovio_px4_interface src/rovio_px4_interface.cpp)
target_include_directories(rovio_px4_interface PUBLIC include)
ament_target_dependencies(rovio_px4_interface 
  rclcpp
  geometry_msgs
  tf2
  tf2_ros)
target_link_libraries(rovio_px4_interface MAVSDK::mavsdk)

# # vio_publisher
# add_executable(vio_publisher src/vio_publisher.cpp)
# target_include_directories(vio_publisher PUBLIC include)
# ament_target_dependencies(vio_publisher 
#   rclcpp
#   geometry_msgs
#   tf2
#   tf2_ros)

# # px4_publisher
# add_executable(px4_publisher src/px4_publisher.cpp)
# target_include_directories(px4_publisher PUBLIC include)
# ament_target_dependencies(px4_publisher 
#   rclcpp
#   geometry_msgs
#   tf2
#   tf2_ros)
# target_link_libraries(vio_px4_interface MAVSDK::mavsdk)


# OLD TARGETS
# vicon px4 publisher
# add_executable(vicon_px4_publisher src/vicon_px4_publisher.cpp)
# target_include_directories(vicon_px4_publisher PUBLIC include)
# ament_target_dependencies(vicon_px4_publisher 
#   rclcpp
#   geometry_msgs
#   osprey_interface)
# target_link_libraries(vicon_px4_publisher 
#   ViconDataStreamSDK_lib 
#   MAVSDK::mavsdk)

# # px4 publisher
# add_executable(px4_publisher src/px4_publisher.cpp)
# target_include_directories(px4_publisher PUBLIC include)
# ament_target_dependencies(px4_publisher 
#   rclcpp
#   osprey_interface)
# target_link_libraries(px4_publisher MAVSDK::mavsdk)

# kimera pose transformer
# add_executable(kimera_pose_transformer src/kimera_pose_transformer.cpp)
# target_include_directories(kimera_pose_transformer PUBLIC 
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# # target_sources(kimera_pose_transformer PUBLIC)
# ament_target_dependencies(kimera_pose_transformer
#   rclcpp
#   tf2
#   nav_msgs
#   osprey_interface)


# INSTALLATION
install(TARGETS
  vicon_px4_interface
  vicon_publisher
  vio_px4_interface
  rovio_px4_interface
  # vio_publisher
  # px4_publisher
  DESTINATION lib/${PROJECT_NAME})


ament_package()