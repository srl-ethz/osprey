#include "rclcpp/rclcpp.hpp"
#include "mission_control/mission_control.hpp"

// NOTE: left angle = front


int main(int argc, char *argv[])
{
  // set required interfaces
  InterfaceList required_interfaces;
  required_interfaces.set_gripper = true;

  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>(required_interfaces);
  if(!rclcpp::ok()) {mission_control_node->shutdown();}
  RCLCPP_INFO(mission_control_node->get_logger(), "Initialization complete. Commencing static gripper test.");
  
  // mission
  mission_control_node->setGripper(0, 0); // close gripper
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait a bit
  mission_control_node->setGripper(90, 90); // open gripper again
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait a bit
  mission_control_node->setGripper(0, 0); // close gripper
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait a bit
  mission_control_node->setGripper(90, 90); // open gripper again
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait a bit

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");
  rclcpp::shutdown();
  return 0;
}