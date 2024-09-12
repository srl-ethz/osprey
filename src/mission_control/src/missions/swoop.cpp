#include "rclcpp/rclcpp.hpp"
#include "mission_control/mission_control.hpp"


std::array<float, 3> home_position = {0.0, 0.0, 0.5};
std::array<float, 3> home_position_low = {0.0, 0.0, 0.15};
std::array<float, 3> home_position_high = {0.0, 0.0, 0.5};

// soft swoop VICON
std::array<float, 3> swoop_start_offset = {-1.0, 0.0, 1.0};
std::array<float, 3> swoop_grasp_offset = {0.05, 0.0, 0.15};
std::array<float, 3> swoop_end_offset = {1.0, 0.0, 1.0};
std::array<float, 3> swoop_land_offset = {1.0, -1.0, 0.5};

// soft swoop VIO
// std::array<float, 3> swoop_start_offset = {0.05, -1.0, 1.0};
// std::array<float, 3> swoop_grasp_offset = {0.05, 0.0, 0.24};
// std::array<float, 3> swoop_end_offset = {0.05, 1.0, 1.0};
// std::array<float, 3> swoop_land_offset = {1.0, 1.0, 0.5};

// aggressive swoop
// std::array<float, 3> start_position = {-0.5, -1.0, 1.5};
// std::array<float, 3> swoop_position = {1.0, -1.0, 0.5};
// std::array<float, 3> end_position = {2.5, -1.0, 1.5};


int main(int argc, char *argv[])
{
  // set required interfaces
  InterfaceList required_interfaces;
  required_interfaces.arm = true;
  required_interfaces.takeoff = true;
  required_interfaces.land = true;
  required_interfaces.go_to_pos = true;
  required_interfaces.set_gripper = true;
  required_interfaces.object_pose = true;

  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>(required_interfaces);
  if(!rclcpp::ok()) {mission_control_node->shutdown();}
  RCLCPP_INFO(mission_control_node->get_logger(), "Initialization complete. Starting mission.");
  // rclcpp::spin_some(mission_control_node);
  // if(!mission_control_node->checkInterfaces(required_interfaces)) {mission_control_node->shutdown();}

  // takeoff
  if(!mission_control_node->arm()) {mission_control_node->shutdown();}
  if(!mission_control_node->takeoff(0.7)) {mission_control_node->shutdown();}

  // mission
  mission_control_node->setGripper(90, 45);
  if(!mission_control_node->go_to_object(swoop_start_offset, 0.0, 4.0, true)){mission_control_node->shutdown();}
  if(!mission_control_node->go_to_object(swoop_grasp_offset, 0.0, 4.0, false)){mission_control_node->shutdown();}
  if(!mission_control_node->go_to_object(swoop_end_offset, 0.0, 0.75, true)){mission_control_node->shutdown();}
  // if necessary, wait
  mission_control_node->setGripper(0, 0);
  if(!mission_control_node->go_to_object(swoop_end_offset, 0.0, 4.0, true)){mission_control_node->shutdown();}
  mission_control_node->setGripper(90, 90); // open gripper
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait a bit :)

  // land
  // if(!mission_control_node->go_to_pos(home_position_high, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  // if(!mission_control_node->go_to_pos(home_position, 0.0, 1.0, true)) {mission_control_node->shutdown();}
  // if(!mission_control_node->go_to_pos(home_position_low, 0.0, 1.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_object(swoop_land_offset, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->land()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");
  rclcpp::shutdown();
  return 0;
}