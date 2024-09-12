#include "rclcpp/rclcpp.hpp"
#include "mission_control/mission_control.hpp"


int main(int argc, char *argv[])
{
  // set required interfaces
  InterfaceList required_interfaces;
  required_interfaces.arm = true;
  required_interfaces.takeoff = true;
  required_interfaces.land = true;
  required_interfaces.go_to_pos = true;
  // required_interfaces.set_gripper = true;
  // required_interfaces.object_pose = true;

  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>(required_interfaces);
  if(!rclcpp::ok()) {mission_control_node->shutdown();}
  RCLCPP_INFO(mission_control_node->get_logger(), "Initialization complete. Starting mission.");

  RCLCPP_INFO(mission_control_node->get_logger(), "Starting mission.");

  // takeoff
  if(!mission_control_node->arm()) {mission_control_node->shutdown();}
  if(!mission_control_node->takeoff(0.7)) {mission_control_node->shutdown();}
  // let quad hover for 5 seconds
  std::this_thread::sleep_for(std::chrono::seconds(5));
  // land
  if(!mission_control_node->land()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");

  rclcpp::shutdown();
  return 0;
}