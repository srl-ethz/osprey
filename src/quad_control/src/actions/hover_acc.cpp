#include "quad_control/quad.hpp"




rclcpp_action::GoalResponse Quad::handleHoverAccGoal(const rclcpp_action::GoalUUID & uuid,
                                                      std::shared_ptr<const HoverAcc::Goal> goal)
{
  (void) uuid;
  (void) goal;

  RCLCPP_INFO(this->get_logger(), "Received hover (acc) goal request.");

  // check if request is valid TODO

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse Quad::handleHoverAccCancel(const std::shared_ptr<HoverAccGoalHandle> goal_handle)
{
  (void) goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received hover (acc) cancel request.");
  // accept in any case
  return rclcpp_action::CancelResponse::ACCEPT;
}


void Quad::handleHoverAccAccepted(const std::shared_ptr<HoverAccGoalHandle> goal_handle)
{
  quad_state_->setState(State::POSITION);

  std::thread{std::bind(&Quad::executeHoverAcc, this, std::placeholders::_1), goal_handle}.detach();
}


void Quad::executeHoverAcc(const std::shared_ptr<HoverAccGoalHandle> goal_handle)
{
  // prepare action variables
  const auto goal = goal_handle->get_goal();
  // auto feedback = std::make_shared<HoverAcc::Feedback>(); no feedback
  auto result = std::make_shared<HoverAcc::Result>();

  // information
  RCLCPP_INFO(this->get_logger(), "Executing hover (acc) for %ds with threshold [%f, %f, %f].",
    goal->time_s,
    goal->pos_threshold_m[0], goal->pos_threshold_m[1], goal->pos_threshold_m[2]);

  // get current position
  std::array<float, 3> initial_position = telemetry_->getPosition();
  RCLCPP_INFO(this->get_logger(), "Initial position: [%f, %f, %f]",
    initial_position[0], initial_position[1], initial_position[2]);

  // timing parameters
  int rate = 50;
  rclcpp::Rate loop_rate(rate);
  int max_iterations = goal->time_s * rate;

  // control loop
  for (int i = 0; i < max_iterations; ++i) {
    if (!rclcpp::ok()) {return;} // check ROS shutdwon
    if (goal_handle->is_canceling()) { // check cancellation
      abortHoverAcc(goal_handle, 30, true);
      return;
    }

    // do control step
    if (!doHoverAccStep(initial_position, goal->pos_threshold_m)) {
      abortHoverAcc(goal_handle, 100, false);
      return;
    }

    loop_rate.sleep();
  }

  // success
  result->return_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Hover (acc) execution successful. (Current position: [%f, %f, %f])",
    telemetry_->getPosition()[0], telemetry_->getPosition()[1], telemetry_->getPosition()[2]);
}


void Quad::abortHoverAcc(const std::shared_ptr<HoverAccGoalHandle> goal_handle, 
                    const int return_code, 
                    bool cancel)
{
  std::array<float, 3> current_position = telemetry_->getPosition();

  mavsdk_wrapper_->sendPositionMessage(current_position); // hold current position

  RCLCPP_WARN(this->get_logger(), "Hover (acc) stopped (%d). Maintaining current position.", 
    return_code);

  auto result = std::make_shared<HoverAcc::Result>();
  result->return_code = return_code;

  if (cancel) {
    goal_handle->canceled(result);
    return;
  }

  goal_handle->abort(result);
  return;
}


bool Quad::doHoverAccStep(const std::array<float, 3> &initial_position,
                          const std::array<float, 3> &pos_threshold_m)
{
  // get current position
  std::array<float, 3> current_position = telemetry_->getPosition();

  // check safety margins
  std::array<float, 3> deviation = {current_position[0] - initial_position[0],
                                    current_position[1] - initial_position[1],
                                    current_position[2] - initial_position[2]};
  if (std::abs(deviation[0]) > pos_threshold_m[0] ||
      std::abs(deviation[1]) > pos_threshold_m[1] ||
      std::abs(deviation[2]) > pos_threshold_m[2]) {
    RCLCPP_INFO(this->get_logger(), "Out of bounds [%f, %f, %f].",
      current_position[0], current_position[1], current_position[2]);
    return false;
  }

  // P control TODO tune
  float k_p = 2.f;
  float k_d = 0.5f;

  std::array<float, 3> acceleration_msg = 
      {- k_p * deviation[0] - k_d * telemetry_->getVelocity()[0],
        - k_p * deviation[1] - k_d * telemetry_->getVelocity()[1],
        - k_p * deviation[2] - k_d * telemetry_->getVelocity()[2]};

  mavsdk_wrapper_->sendAccelerationMessage(acceleration_msg);
  return true;
}