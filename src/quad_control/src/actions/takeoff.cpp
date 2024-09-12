#include "quad_control/quad.hpp"




rclcpp_action::GoalResponse Quad::handleTakeoffGoal(const rclcpp_action::GoalUUID & uuid,
                                                      std::shared_ptr<const Takeoff::Goal> goal)
{
  (void) uuid;
  (void) goal;

  RCLCPP_INFO(this->get_logger(), "Received takeoff goal request.");

  // check if request is valid
  if (!quad_state_->isValidTransition(State::TAKEOFF)) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  // TODO #33 Validate Requested Takeoff Height

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse Quad::handleTakeoffCancel(const std::shared_ptr<TakeoffGoalHandle> goal_handle)
{
  (void) goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received takeoff cancel request.");

  // accept in any case
  return rclcpp_action::CancelResponse::ACCEPT;
}


void Quad::handleTakeoffAccepted(const std::shared_ptr<TakeoffGoalHandle> goal_handle)
{
  quad_state_->setState(State::TAKEOFF);

  std::thread{std::bind(&Quad::executeTakeoff, this, std::placeholders::_1), goal_handle}.detach();
}


void Quad::executeTakeoff(const std::shared_ptr<TakeoffGoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing takeoff.");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Takeoff::Feedback>();
  auto result = std::make_shared<Takeoff::Result>();

  rclcpp::Rate loop_rate_20(20);
  rclcpp::Rate loop_rate_100(100);

  // get current position to determine ground height
  float ground_height = telemetry_->getPosition()[2];

  // set target position after takeoff
  std::array<float,3> target_position = {0.0f, 0.0f, goal->height};




  // arming stage

  feedback->stage = 0;
  goal_handle->publish_feedback(feedback);

  // TODO should we send a 'safety' arming request here?
  mavsdk_wrapper_->sendArmRequest();

  // PX4 only enables offboard mode after receiving offboard commands for over a second.
  // Therefore, start stream here for 1 second, and continue stream throughout function execution
  for (size_t i = 0; i < 10; ++i) {
    // check ROS shutdwon
    if (!rclcpp::ok()) {return;}

    // check cancellation
    if (goal_handle->is_canceling()) {
      abortTakeoff(goal_handle, 30, true);
      return;
    }

    mavsdk_wrapper_->sendPositionMessage(target_position);

    loop_rate_20.sleep();
  }




  // takeoff stage
  
  // send takeoff command
  int mavsdk_action_result = mavsdk_wrapper_->sendTakeoffRequest();
  // abort if mavsdk request was rejected
  if (mavsdk_action_result != 1) {
    abortTakeoff(goal_handle, 300+mavsdk_action_result);
    return;
  }

  feedback->stage = 1;
  goal_handle->publish_feedback(feedback);

  // wait until drone is 5cm above ground (airborne)
  float height_threshold = ground_height + 0.05;

  for (size_t i = 0; telemetry_->getPosition()[2] < height_threshold; ++i) {
    // check ROS shutdwon
    if (!rclcpp::ok()) {return;}

    // check cancellation
    if (goal_handle->is_canceling()) {
      abortTakeoff(goal_handle, 30, true);
      return;
    }

    // timeout after 5 seconds
    if (i == 5000) {
      abortTakeoff(goal_handle, 104);
      return;
    }

    // continue stream of offboard commands
    mavsdk_wrapper_->sendPositionMessage(target_position);

    loop_rate_20.sleep();
  }


  // wait until drone reaches correct height for switching into offboard (considering control delay)
  // threshold = requested_height - v_z * (control_delay)
  // v_z = 1.5 m/s
  // control_delay = 0.1 s
  height_threshold = goal->height - (1.5 * 0.1);

  for (size_t i = 0; telemetry_->getPosition()[2] < height_threshold; ++i) {
    // check ROS shutdwon
    if (!rclcpp::ok()) {return;}

    // check cancellation
    if (goal_handle->is_canceling()) {
      abortTakeoff(goal_handle, 30, true);
      return;
    }

    // timeout after 10 seconds
    if (i == 10000) {
      abortTakeoff(goal_handle, 104);
      return;
    }

    // continue stream of offboard commands
    mavsdk_wrapper_->sendPositionMessage(target_position);

    loop_rate_100.sleep();
  }


  // stabilization stage

  int mavsdk_offboard_result = mavsdk_wrapper_->sendOffboardRequest();

  // abort if mavsdk request was rejected
  if (mavsdk_offboard_result != 1) {
    abortTakeoff(goal_handle, 300 + mavsdk_offboard_result);
      return;
  }

  feedback->stage = 2;
  goal_handle->publish_feedback(feedback);

  // continue sending offboard message stream for 2s to stabilize drone
  for (size_t i = 0; i < 40; ++i) {
    // check ROS shutdwon
    if (!rclcpp::ok()) {return;}

    // check cancellation
    if (goal_handle->is_canceling()) {
      abortTakeoff(goal_handle, 30, true);
      return;
    }

    mavsdk_wrapper_->sendPositionMessage(target_position);

    loop_rate_20.sleep();
  }


  // success
  result->return_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Takeoff execution successful.");

}


void Quad::abortTakeoff(const std::shared_ptr<TakeoffGoalHandle> goal_handle, 
                        const int return_code, 
                        bool cancel)
{
  int mavsdk_action_result = mavsdk_wrapper_->sendLandRequest();

  RCLCPP_WARN(this->get_logger(), "Takeoff stopped (%d). Land request: %s.", 
    return_code,
    actionResultToString(mavsdk_action_result).c_str());

  auto result = std::make_shared<Takeoff::Result>();
  result->return_code = return_code;

  if (cancel) {
    goal_handle->canceled(result);
    return;
  }

  goal_handle->abort(result);
  return;
}