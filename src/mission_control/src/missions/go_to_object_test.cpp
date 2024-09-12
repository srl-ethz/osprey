#include "rclcpp/rclcpp.hpp"
#include "mission_control/mission_control.hpp"

std::array<float, 3> land_position = {1.5, 0.0, 0.3};
std::array<float, 3> land_position_low = {1.5, 0.0, 0.15};
std::array<float, 3> land_position_high = {1.5, 0.0, 0.5};


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
  if(!mission_control_node->takeoff(0.7)) {mission_control_node->shutdown();}
   std::this_thread::sleep_for(std::chrono::milliseconds(3000)); // wait a bit :)

  // mission
  if(!mission_control_node->go_to_object({0.0, 0.0, 0.5}, 0.0, 4.0, true)){mission_control_node->shutdown();}
  // grasp object
  if(!mission_control_node->go_to_object({0.0, 0, 0.3}, 0.0, 4.0, true)){mission_control_node->shutdown();}

  auto curr_drone_pose = mission_control_node->current_drone_pose_;
  auto curr_object_pose = mission_control_node->current_object_pose_;
  
  float x_diff = curr_drone_pose->pose.position.x - curr_object_pose->pose.position.x;
  float y_diff = curr_drone_pose->pose.position.y - curr_object_pose->pose.position.y;
  float z_diff = curr_drone_pose->pose.position.z - curr_object_pose->pose.position.z;

  RCLCPP_WARN(mission_control_node->get_logger(), "x diff(%f).", x_diff);
  RCLCPP_WARN(mission_control_node->get_logger(), "y diff(%f).", y_diff);
  RCLCPP_WARN(mission_control_node->get_logger(), "z diff(%f).", z_diff);

  mission_control_node->setGripper(0, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait a bit :)
  if(!mission_control_node->go_to_object({0.5, 0.0, 0.8}, 0.0, 4.0, true)){mission_control_node->shutdown();}
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait a bit :)
  mission_control_node->setGripper(90, 90); // open gripper
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait a bit :)

  // land
  if(!mission_control_node->go_to_pos(land_position_high, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(land_position, 0.0, 1.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(land_position_low, 0.0, 1.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->land()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");
  rclcpp::shutdown();
  return 0;
}
