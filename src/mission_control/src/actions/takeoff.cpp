#include "mission_control/mission_control.hpp"




bool MissionControl::takeoff(const float altitude) {
  // TODO check quad state

  // check if service is available TODO
  if (!act_takeoff_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Takeoff action not found.");
    return false;
  }

  // create first goal
  auto goal = Takeoff::Goal(); // create goal
  goal.height = altitude; // set goal
  auto goal_handle_future = act_takeoff_->async_send_goal(goal); // request goal

  // wait until goal was processed
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), goal_handle_future, std::chrono::seconds(1)) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Takeoff action failed (ROS error).");
    return false;
  }

  // check if goal was accepted
  rclcpp_action::ClientGoalHandle<Takeoff>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Takeoff goal rejected by server.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Taking off.");

  // waiting until action is completed
  auto result_future = act_takeoff_->async_get_result(goal_handle);
  // check return code
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), result_future, std::chrono::seconds(20)) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Takeoff action failed (ROS error).");
    return false;
  }
  
  // evaluate response
  auto result_wrapper = result_future.get();
  if (result_wrapper.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(this->get_logger(), "Takeoff action failed (%i).", result_wrapper.result->return_code);
    return false;
  }


  RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
  return true;
}