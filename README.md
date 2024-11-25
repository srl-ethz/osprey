We present the accompanying code and hardware schematics to *[An Open-Source Soft Robotic Platform for Autonomous Aerial Manipulation in the Wild](https://openreview.net/forum?id=SfaB20rjVo)*. This workspace contains all required ROS 2 packages for the platform.

# Workspace Structure

## Environment

Our software stack was developed for ROS2 Foxy. To install ROS2 Foxy, follow the official [installation guide](https://docs.ros.org/en/foxy/Installation.html). We use both Python and C++ for our software stack. Your Python environment for ROS should have the following packages:

### Python

You will need a Python environment for ROS and a separate environment for running segmentation, which is independent of ROS. This makes it easier to try different segmentation algorithms, as you can freely switch the Python version and other dependencies.

```bash
pip install open3d imagezmq matplotlib spectacularAI
```
Our segmentation node runs outside of ROS to make it easier to develop on with conda environments. To set up the environment for the segmentation node:

```bash
mamba env create -f sam2_environment.yml
```

In case it is needed, backwards compatibility to ROS1 can be achieved using the [ros1_bridge](https://github.com/ros2/ros1_bridge).

### Other setup

To use a Vicon motion capture system, if you have it, follow the installation guide of the `ros2-vicon-receiver` submodule, which you can find [here](https://github.com/srl-ethz/ros2-vicon-receiver/tree/f99950bee0bbc09b79a0bd06add523d4861433b9).

You should also install [MAVlink](https://mavlink.io/en/) for communication.

## CAD Files
CAD files were created with NX Siemens. For best compatibility use NX 1988 to open the files. The assembly file is located at cad_parts/098082_02.prt

## Packages

### Quad Control
Control interface to the PX4 using mavlink.

- API for interactions with the PX4

### Vicon
Interface to a motion capture system and to forward pose estimation data to the PX4 (from both motion capture and SLAM systems).

- Publish motion capture pose data from the Vicon system to the ROS 2 network
- Forward motion capture pose data from the Vicon system to the PX4 controller
- Publish SLAM data to the PX4 controller
- Publish telemetry data from the PX4 controller to the ROS 2 network

### Spectacular VIO
ROS2 wrapper for [Spectacular AI](https://www.spectacularai.com/) SLAM/VIO system using an OAK-D camera.

### Gripper Control
ROS2 node that receives gripper commands and forwards them over serial to an Arduino. Also includes Arduino file, which receives commands over serial and controls the servos of the gripper.

- API for interactions with the gripper. 

### OSPREY Interface

Wrapper for high-level actions.

- Custom ROS interface definitions for the OSPREY workspace.

## Nodes

These are the nodes you will usually run:

- quad_control
- mission_control
- spectacular_node
- vio_px4_interface
- gripper_control
- video_sender_node

Outside of ROS:
- sam2_detection.py

## Interfaces

Services
- `arm`
- `land`
- `set_gripper`

Actions
- `takeoff`
- `go_to_pos`
- `hover_acc`

# Conventions

## CMakeLists.txt

```CMake
cmake_minimum_required(VERSION 3.13)

project(my_project)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
# ros custom
find_package(custom_ros_package REQUIRED)
# external
find_package(external_package REQUIRED)


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
# my_target
add_executable(my_target src/my_target.cpp)
ament_target_dependencies(my_target 
  ros_package)
target_link_libraries(my_target external_package)

# another_target
add_executable(another_target src/another_target.cpp)


# INSTALLATION
install(TARGETS
  my_target
  another_target
  DESTINATION lib/${PROJECT_NAME})


ament_package()
```


# Source, Build, Install, Launch

## Source

Run on every new shell to have access to ros2 commands
```bash
source /opt/ros/foxy/setup.bash
```

## Build

Build all packages, run the build command within the workspace's parent directory
```bash
colcon build
```
To build only a specific package, run
```bash
colcon build --packages-select <package_name>
```

## Install
For using executables, run the installation script from inside the workspace's parent directory
```bash
. install/local_setup.bash
```

## Launch
To launch a ros-node, run
```bash
ros2 run <package_name> <my_node>
```

To set the verbosity level to debug, run
```bash
ros2 run <package_name> <my_node> --log-level debug
```

## Verbosity

To set the default verbosity level, run
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```


# Create new Packages and Nodes

## Packages

Within osprey/src run
```bash
ros2 pkg create <package_name> --build-type ament_cmake --dependencies rclcpp 
```
Note: '--dependencies rclcpp' will automatically include rclcpp as a dependency in package.xml and CMakeLists.txt.

## Nodes

### XML
Add dependencies in package.xml
```xml
<depend>rclcpp</depend>
```

### CMake
In CMakeLists.txt:
- add dependencies 
  ```CMake
  find_package(rclcpp REQUIRED)
  ```

- add target/executable
  ```CMake
  add_executable(my_programm src/my_programm_src.cpp)
  ```

- link libraries to target
  ```CMake
  ament_target_dependencies(my_programm rclcpp) # link ros dependencies
  target_link_libraries(my_programm external_package) # link standard dependencies
  ```

- add target to installation list
  ```CMake
  install(TARGETS
    my_programm
    DESTINATION lib/${PROJECT_NAME})
  ```

### CPP
Include header files
```CPP
#include "rclcpp/rclcpp.hpp" // ros2 client library
#include "std_msgs/msg/string.hpp" // message and service definitions
```

# Interfaces

## List and Call Interfaces from Commandline

- List all active topics and services
  ```bash
  ros2 topic list
  ros2 service list
  ```

- Show type of a topic or service
  ```bash
  ros2 service type /topic_or_service_name
  ```

- Show structure of a type
  ```bash
  ros2 interface show <TypeName>
  ```

- Call service from commandline
  ```bash
  ros2 service call /my_service <TypeName> "{request_variable_1: value, request_variable_2: value}"
  ```

## Custom Interfaces

- Available messages and services: [ros2 common interfaces](https://github.com/ros2/common_interfaces)
- Available types: [ros2 built in types](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html)

Message and service should be provided in separate interface package. \
Names **must** follow the *CamelCased* convention. \
Definitions must be placed in directories called msg and srv respectively. \

For services, define request and response variables in the **/srv/MyService.srv** file using the following structure:
```srv
type request_variable
---
type response_variable
```

Add the required dependency in the CMakeLists.txt.
```CMake
# required package for generating custom services
find_package(rosidl_default_generators REQUIRED)

# define which services should be generated
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyService.srv"
  DEPENDENCIES <ros_msg> # If custom messages depend on ros_msgs
)
```

Add the required dependency in the package.xml.
```XML
<!-- build dependency -->
<build_depend>rosidl_default_generators</build_depend>

<!-- runtime dependency -->
<exec_depend>rosidl_default_runtime</exec_depend> 

<!-- name of dependency group to which the package belongs -->
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Testing custom interface
Build and install package, then run
```bash
ros2 interface show my_package/msg/MyMessage
ros2 interface show my_package/srv/MyService
```

# Citation

If you find this code useful for your research, please consider citing our paper:

```
@inproceedings{
  bauer2024an,
  title={An Open-Source Soft Robotic Platform for Autonomous Aerial Manipulation in the Wild},
  author={Erik Bauer and Marc Bl{\"o}chlinger and Pascal Strauch and Arman Raayatsanati and Cavelti Curdin and Robert K. Katzschmann},
  booktitle={8th Annual Conference on Robot Learning},
  year={2024},
  url={https://openreview.net/forum?id=SfaB20rjVo}
}
```
