cmake_minimum_required(VERSION 3.8)
project(osprey_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Pose.msg"
  "msg/Velocity.msg"
  "msg/Acceleration.msg"
  "msg/Detection.msg"
)
set(srv_files
  "srv/Trigger.srv"
  "srv/GoToPos.srv"
  "srv/QuadStatus.srv"
  "srv/SetGripper.srv"
)
set(action_files
  "action/Takeoff.action"
  "action/GoToPos.action"
  "action/HoverAcc.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  ${action_files}
)

ament_package()