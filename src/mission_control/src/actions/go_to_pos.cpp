#include "mission_control/mission_control.hpp"




bool MissionControl::go_to_pos(const std::array<float, 3> &pos, const float yaw, const float timeout_s, const bool wait) {
  // TODO check quad state

  // check if service is available TODO
  if (!act_goToPos_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "GoToPos action not found.");
    return false;
  }

  // create first goal
  auto goal = GoToPos::Goal(); // create goal
  // set goal
  goal.x_m = pos[0];
  goal.y_m = pos[1];
  goal.z_m = pos[2];
  goal.yaw_deg = yaw;
  goal.timeout_s = timeout_s;
  goal.wait = wait;
  auto goal_handle_future = act_goToPos_->async_send_goal(goal); // request goal

  // wait until goal was processed
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), goal_handle_future, std::chrono::seconds(1)) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "GoToPos action failed (ROS error).");
    return false;
  }

  // check if goal was accepted
  rclcpp_action::ClientGoalHandle<GoToPos>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "GoToPos goal rejected by server.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Going to position [%f, %f, %f | %f] (timeout=%f, wait=%d).",
    pos[0], pos[1], pos[2], yaw, timeout_s, wait);

  // waiting until action is completed
  auto result_future = act_goToPos_->async_get_result(goal_handle);
  // check return code
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), result_future, std::chrono::seconds(20)) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "GoToPos action failed (ROS error).");
    return false;
  }
  
  // evaluate response
  auto result_wrapper = result_future.get();
  if (result_wrapper.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(this->get_logger(), "GoToPos action failed (%i).", result_wrapper.result->return_code);
    return false;
  }

  
  RCLCPP_INFO(this->get_logger(), "GoToPos successful.");
  return true;
}

bool MissionControl::go_to_object(const std::array<float, 3> &offset, 
                                  const float yaw, 
                                  const float timeout_s, 
                                  const bool wait)
{
  RCLCPP_INFO(this->get_logger(), "Going to object with offsets [%f, %f, %f] (timeout=%fs, wait=%d).",
    offset[0], offset[1], offset[2], timeout_s, wait);

  float target_x = object_telemetry_->getPosition()[0] + offset[0];
  float target_y = object_telemetry_->getPosition()[1] + offset[1];
  float target_z = object_telemetry_->getPosition()[2] + offset[2];

  // check if object position is within the following bounds:
  // x: [-0.5, 6]
  // y: [-4, 0.5]
  // z: [-1.5, 2.0]
  std::array<float, 2> x_bounds = {-0.5, 6};
  std::array<float, 2> y_bounds = {-4.0, 0.5};
  std::array<float, 2> z_bounds = {-1.5, 2.0};

  if (target_x < x_bounds[0] || target_x > x_bounds[1]) {
    RCLCPP_ERROR(this->get_logger(), "Object position out of bounds (x_ref: %f).", target_x);
    return false;
  }
  if (target_y < y_bounds[0] || target_y > y_bounds[1]) {
    RCLCPP_ERROR(this->get_logger(), "Object position out of bounds (y_ref: %f).", target_y);
    return false;
  }
  if (target_z < z_bounds[0] || target_z > z_bounds[1]) {
    RCLCPP_ERROR(this->get_logger(), "Object position out of bounds (z_ref: %f).", target_z);
    return false;
  }

  std::array<float, 3> target_position = {target_x, target_y, target_z};
  return go_to_pos(target_position, yaw, timeout_s, wait);
}