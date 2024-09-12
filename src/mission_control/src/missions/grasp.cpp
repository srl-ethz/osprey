#include "rclcpp/rclcpp.hpp"
#include "mission_control/mission_control.hpp"


std::array<float, 3> home_position = {0.0, 0.0, 0.5};
std::array<float, 3> home_position_low = {0.0, 0.0, 0.15};
std::array<float, 3> home_position_high = {0.0, 0.0, 0.7};

std::array<float, 3> hover_offset = {0.0, 0.0, 0.7};
std::array<float, 3> approach_offset = {0.0, 0.0, 0.5};
std::array<float, 3> grasp_offset = {0.0, 0.0, 0.3};
std::array<float, 3> drop_offset = {0.5, 0.0, 0.7};


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

  // takeoff
  if(!mission_control_node->arm()) {mission_control_node->shutdown();}
  if(!mission_control_node->takeoff(0.6)) {mission_control_node->shutdown();}
  mission_control_node->setGripper(90, 90); // open gripper

  // mission
  if(!mission_control_node->go_to_object(hover_offset, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_object(approach_offset, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_object(grasp_offset, 0.0, 3.0, true)) {mission_control_node->shutdown();}
  mission_control_node->setGripper(0, 0); // close gripper
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait a bit
  if(!mission_control_node->go_to_object(hover_offset, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_object(drop_offset, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  mission_control_node->setGripper(90, 90); // open gripper again
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait a bit

  // land
  if(!mission_control_node->go_to_pos(home_position_high, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(home_position, 0.0, 1.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(home_position_low, 0.0, 1.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->land()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");
  rclcpp::shutdown();
  return 0;
}