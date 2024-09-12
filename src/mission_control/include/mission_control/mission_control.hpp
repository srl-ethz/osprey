#pragma once

#include <string>

// ros default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// custom
#include "mission_control/telemetry.hpp"

// interfaces
// #include "osprey_interface/msg/pose.hpp"
#include "osprey_interface/srv/trigger.hpp"
#include "osprey_interface/srv/set_gripper.hpp"
#include "osprey_interface/action/takeoff.hpp"
#include "osprey_interface/action/go_to_pos.hpp"
#include "osprey_interface/action/hover_acc.hpp"


struct InterfaceList
{
  bool arm{false};
  bool takeoff{false};
  bool land{false};
  bool go_to_pos{false};
  bool set_gripper{false};
  bool object_pose{false};
};


class MissionControl : public rclcpp::Node
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Trigger = osprey_interface::srv::Trigger;
  using SetGripper = osprey_interface::srv::SetGripper;
  using Takeoff = osprey_interface::action::Takeoff;
  using TakeoffGoalHandle = rclcpp_action::ClientGoalHandle<Takeoff>;
  using GoToPos = osprey_interface::action::GoToPos;
  using GoToPosGoalHandle = rclcpp_action::ClientGoalHandle<GoToPos>;
  using HoverAcc = osprey_interface::action::HoverAcc;
  using HoverAccGoalHandle = rclcpp_action::ServerGoalHandle<HoverAcc>;

  MissionControl(InterfaceList &interfaces);
  ~MissionControl() {};

  // API
  // check if all requested interfaces are available
  bool checkInterfaces(InterfaceList &interfaces);
  bool arm();
  bool setGripper(float front_angle_deg, float back_angle_deg, int mode = 0);
  bool takeoff(const float altitude);
  bool land();
  bool go_to_pos(const std::array<float, 3> &pos, const float yaw, const float timeout_s, const bool wait = false);
  bool go_to_object(const std::array<float, 3> &offset, const float yaw, const float timeout_s, const bool wait = false);
  bool hover_acc(const std::array<float, 3> &threshold_m, int time_s);

  // helper functions
  void shutdown();
  
  // public for now for easy access
  PoseStamped::SharedPtr current_object_pose_;
  PoseStamped::SharedPtr current_drone_pose_;
  size_t object_telemetry_count_{0};
  bool update_object_pose_{true};

private:
  // TODO add quad state
  std::shared_ptr<Telemetry> object_telemetry_;

  // interface clients
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_object_pose_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_drone_pose_;
  rclcpp::Client<Trigger>::SharedPtr srv_arm_;
  rclcpp::Client<Trigger>::SharedPtr srv_land_;
  rclcpp::Client<SetGripper>::SharedPtr srv_set_gripper_;
  rclcpp_action::Client<Takeoff>::SharedPtr act_takeoff_;
  rclcpp_action::Client<GoToPos>::SharedPtr act_goToPos_;
  rclcpp_action::Client<HoverAcc>::SharedPtr act_hoverAcc_;

  // subscriptions
  void objectPoseCallback(const PoseStamped::SharedPtr msg);
  void dronePoseCallback(const PoseStamped::SharedPtr msg);

  // helpers
  bool isValidPosition(const std::array<float,3>& position, float threshold = 1e-4);
};