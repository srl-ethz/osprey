#include "rclcpp/rclcpp.hpp"

#include "mission_control/mission_control.hpp"




std::array<float, 3> home_position = {0.0, 0.0, 0.7};
std::array<float, 3> home_position_low = {0.0, 0.0, 0.4};


int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>();

  if(!rclcpp::ok()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Starting mission.");

  if(!mission_control_node->arm()) {mission_control_node->shutdown();}

  if(!rclcpp::ok()) {mission_control_node->shutdown(); }

  if(!mission_control_node->takeoff(0.7)) {mission_control_node->shutdown();}
  // sleep for 5s
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // let quad hover at current position for 10s
  std::array<float, 3> threshold = {0.3, 0.3, 0.3};
  mission_control_node->hover_acc(threshold, 5);

  // land
  if(!mission_control_node->go_to_pos(home_position, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(home_position_low, 0.0, 2.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->land()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");

  rclcpp::shutdown();
  return 0;
}