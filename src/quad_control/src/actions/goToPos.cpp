#include "quad_control/quad.hpp"




rclcpp_action::GoalResponse Quad::handleGoToPosGoal(const rclcpp_action::GoalUUID & uuid,
                                                      std::shared_ptr<const GoToPos::Goal> goal)
{
  (void) uuid;
  (void) goal;

  RCLCPP_INFO(this->get_logger(), "Received goToPos goal request.");

  // check if request is valid TODO

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse Quad::handleGoToPosCancel(const std::shared_ptr<GoToPosGoalHandle> goal_handle)
{
  (void) goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received goToPos cancel request.");

  // accept in any case
  return rclcpp_action::CancelResponse::ACCEPT;
}


void Quad::handleGoToPosAccepted(const std::shared_ptr<GoToPosGoalHandle> goal_handle)
{
  quad_state_->setState(State::POSITION);

  std::thread{std::bind(&Quad::executeGoToPos, this, std::placeholders::_1), goal_handle}.detach();
}


void Quad::executeGoToPos(const std::shared_ptr<GoToPosGoalHandle> goal_handle)
{

  int rate = 30;

  const float threshold = 0.60;
  // const float x_threshold = 0.0;
  const float z_threshold = 0.4;
  int in_threshold_iterations = 0;

  // be stable in threshold for 1 second
  int target_iterations_in_threshold = 0;

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GoToPos::Feedback>();
  auto result = std::make_shared<GoToPos::Result>();

  RCLCPP_INFO(this->get_logger(), "Executing goToPos [%f, %f, %f, yaw=%d, time=%d, wait=%d].",
    goal->x_m, goal->y_m, goal->z_m, goal->yaw_deg, goal->timeout_s, (int)goal->wait);

  rclcpp::Rate loop_rate(rate);
  int max_iterations = goal->timeout_s * rate;

  std::array<float, 3> position_msg = {goal->x_m, goal->y_m, goal->z_m};


  for (int i = 0; i < max_iterations; ++i) {
    // check ROS shutdwon
    if (!rclcpp::ok()) {return;}

    // check cancellation
    if (goal_handle->is_canceling()) {
      abortGoToPos(goal_handle, 30, true);
      return;
    }

    // TODO check if goal is reached within allowed tolerance

    mavsdk_wrapper_->sendPositionMessage(position_msg, goal->yaw_deg);

    std::array<float, 3> curr_ned_pos = telemetry_->getPosition();

    float x_diff = fabs(curr_ned_pos[0] - position_msg[0]);
    float y_diff = fabs(curr_ned_pos[1] - position_msg[1]);
    float z_diff = fabs(curr_ned_pos[2] - position_msg[2]);

    float x_diff_sq = x_diff * x_diff;
    float y_diff_sq = y_diff * y_diff;

    // RCLCPP_INFO(this->get_logger(), "Target position  xy error %f.",
    // sqrt(x_diff_sq + y_diff_sq));
    // RCLCPP_INFO(this->get_logger(), "Target position z error %f.",
    // z_diff);

    bool xy_thresh_reached = sqrt(x_diff_sq + y_diff_sq) < threshold;
    bool z_thresh_reached = z_diff < z_threshold; 

    if (xy_thresh_reached && z_thresh_reached) {
      in_threshold_iterations++;
      // RCLCPP_INFO(this->get_logger(), "Target position within threshold.");
    } else {
      in_threshold_iterations = 0;
    }

    if (!goal->wait && (in_threshold_iterations > target_iterations_in_threshold) ) {
      RCLCPP_INFO(this->get_logger(), "Reached target position early, moving on.");
      break;
    }

    loop_rate.sleep();
  }


  // success
  result->return_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "GoToPos execution finished.");

}


void Quad::abortGoToPos(const std::shared_ptr<GoToPosGoalHandle> goal_handle, 
                    const int return_code, 
                    bool cancel)
{
  // get current position
  std::array<float, 3> position_msg = telemetry_->getPosition();

  // send position message with current position to stop
  mavsdk_wrapper_->sendPositionMessage(position_msg); // TODO consider yaw

  RCLCPP_WARN(this->get_logger(), "GoToPos stopped (%d). Maintaining current position.", 
    return_code);

  auto result = std::make_shared<GoToPos::Result>();
  result->return_code = return_code;

  if (cancel) {
    goal_handle->canceled(result);
    return;
  }

  goal_handle->abort(result);
  return;
}