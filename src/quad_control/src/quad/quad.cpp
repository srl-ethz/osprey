#include "quad_control/quad.hpp"

// TODO read these values from a yaml file
const int position_pub_interval = 50;
const std::string TOPIC_POSE = "/px4/pose/ned";
const std::string TOPIC_VEL = "px4_vel_nwu"; // currently unused

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// Construction

// Constructor
Quad::Quad(const std::string &port) : Node("quad_control") {
  using namespace std::placeholders;
  RCLCPP_INFO(this->get_logger(), "Initializing...");

  // initialize modules
  quad_state_ = std::make_shared<QuadState>();
  mavsdk_wrapper_ = std::make_shared<MavsdkWrapper>();
  telemetry_ = std::make_shared<Telemetry>();
  if (mavsdk_wrapper_->initialize(port)) {
    RCLCPP_ERROR(this->get_logger(), "MavsdkWrapper initialization failed.");
    rclcpp::shutdown();
    return;
  }


  // subscriptions
  sub_pose_ =  this->create_subscription<geometry_msgs::msg::PoseStamped>(
                TOPIC_POSE, 10, std::bind(&Quad::pose_callback, this, _1));
  // if (sub_pose_->get_publisher_count() == 0) {
  //   RCLCPP_ERROR(this->get_logger(), "No publishers on topic [%s].", TOPIC_POSE.c_str());
  //   rclcpp::shutdown();
  //   return;
  // }
  // not implemented yet
  // sub_vel_ = this->create_subscription<osprey_interface::msg::Velocity>(
  //             TOPIC_VEL, 10, std::bind(&Quad::vel_callback, this, _1));


  // service servers
  srv_arm_ = this->create_service<Trigger>(
              "arm", std::bind(&Quad::arm, this, _1, _2));
  srv_land_ = this->create_service<Trigger>(
              "land", std::bind(&Quad::land, this, _1, _2));


  // action servers
  act_takeoff_ = rclcpp_action::create_server<Takeoff>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "takeoff",
    std::bind(&Quad::handleTakeoffGoal, this, _1, _2),
    std::bind(&Quad::handleTakeoffCancel, this, _1),
    std::bind(&Quad::handleTakeoffAccepted, this, _1));
  act_goToPos_ = rclcpp_action::create_server<GoToPos>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "go_to_pos",
    std::bind(&Quad::handleGoToPosGoal, this, _1, _2),
    std::bind(&Quad::handleGoToPosCancel, this, _1),
    std::bind(&Quad::handleGoToPosAccepted, this, _1));
  act_hoverAcc_ = rclcpp_action::create_server<HoverAcc>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "hover_acc",
    std::bind(&Quad::handleHoverAccGoal, this, _1, _2),
    std::bind(&Quad::handleHoverAccCancel, this, _1),
    std::bind(&Quad::handleHoverAccAccepted, this, _1));


  quad_state_->setState(State::INITIALIZED);
  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////// Subscriptions

void Quad::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  telemetry_->setPosition({(float)msg->pose.position.x, 
                           (float)-msg->pose.position.y, 
                           (float)-msg->pose.position.z});
  // currently don't set any attitude
  // telemetry_->setAttitude({msg->roll_deg, msg->pitch_deg, msg->yaw_deg});
}

void Quad::vel_callback(const Velocity::SharedPtr msg)
{
  telemetry_->setVelocity({msg->x_m_s, msg->y_m_s, msg->z_m_s});
}




////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////// Services

void Quad::arm(std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response) {
  (void)request; // suppress unused variable warning

  RCLCPP_INFO(this->get_logger(), "Received arm request.");

  // check if proper state
  if (!quad_state_->isValidTransition(State::ARMED)) {
    response->result = 201; // request not feasible
    return;
  }

  const int mavsdk_result = mavsdk_wrapper_->sendArmRequest();

  if (mavsdk_result == 1) {
    response->result = 0; // success
    quad_state_->setState(State::ARMED);
  } else {
    response->result = mavsdk_result + 300; // mavsdk error
  }
  
  // debug
  RCLCPP_INFO(this->get_logger(), "Arm result: [%d]", response->result);
}


void Quad::land(std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response) {
  (void)request; // suppress unused variable warning

  RCLCPP_INFO(this->get_logger(), "Received land request.");

  // check if proper state TODO

  const int mavsdk_result = mavsdk_wrapper_->sendLandRequest();

  if (mavsdk_result == 1) {
    response->result = 0; // success
    quad_state_->setState(State::INITIALIZED); // TODO
  } else {
    response->result = mavsdk_result + 300; // mavsdk error
  }
  
  // debug
  RCLCPP_INFO(this->get_logger(), "Land result: [%d]", response->result);
}
