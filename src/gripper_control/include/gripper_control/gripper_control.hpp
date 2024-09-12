#pragma once

#include "rclcpp/rclcpp.hpp"
#include "osprey_interface/srv/set_gripper.hpp"
#include "serialib.h"




class Gripper : public rclcpp::Node
{
public:
  using SetGripper = osprey_interface::srv::SetGripper;

  // serial port to arduino: /dev/ttyXXXY
  Gripper(const std::string &port);
  ~Gripper() {};

private:
  const float threshold_ = 70;
  serialib serial_;
  // data fields: mode, threshold, anlge_front, angle_back
  std::array<unsigned char, 4> cmd_ = {0, 50, 90, 90};
  rclcpp::Service<SetGripper>::SharedPtr srv_set_gripper_;

  void setGripper(const std::shared_ptr<SetGripper::Request> request,
                  std::shared_ptr<SetGripper::Response> response);
  
  // unsigned char cmd_[] = {90, 90}; // serial data packet
};