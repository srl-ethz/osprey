#include "mission_control/mission_control.hpp"


const std::string TOPIC_DRONE_POSE = "/slam/odometry";
// const std::string TOPIC_OBJECT = "/vicon/srl_object/srl_object"; // full vicon
// const std::string TOPIC_OBJECT = "/object/vicon/pose_ned"; // hybrid vicon-vio
const std::string TOPIC_OBJECT = "/object_detection/target/world_nwu"; // no vicon
const std::string ACT_LABEL_TAKEOFF = "takeoff";
const std::string ACT_LABEL_GOTOPOS = "go_to_pos";
const std::string ACT_LABEL_HOVERACC = "hover_acc";


MissionControl::MissionControl(InterfaceList &interfaces) : Node("mission_control")
{
  using namespace std::placeholders;

  RCLCPP_INFO(this->get_logger(), "Initializing mission control node...");

  object_telemetry_ = std::make_shared<Telemetry>();

  // subscriptions
  sub_object_pose_ = this->create_subscription<PoseStamped>(
    TOPIC_OBJECT, 10, std::bind(&MissionControl::objectPoseCallback, this, _1));
  sub_drone_pose_ = this->create_subscription<PoseStamped>(
    TOPIC_DRONE_POSE, 10, std::bind(&MissionControl::dronePoseCallback, this, _1));


  // service clients
  srv_arm_ = this->create_client<Trigger>("arm");
  srv_land_ = this->create_client<Trigger>("land");
  srv_set_gripper_ = this->create_client<SetGripper>("set_gripper");

  // action clients
  act_takeoff_ = rclcpp_action::create_client<Takeoff>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    ACT_LABEL_TAKEOFF);

  act_goToPos_ = rclcpp_action::create_client<GoToPos>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    ACT_LABEL_GOTOPOS);

  act_hoverAcc_ = rclcpp_action::create_client<HoverAcc>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    ACT_LABEL_HOVERACC);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait for interfaces to initialize
  rclcpp::spin_some(this->get_node_base_interface());
  if(!checkInterfaces(interfaces)) {shutdown();}

  RCLCPP_INFO(this->get_logger(), "Initialization complete.");
}


bool MissionControl::checkInterfaces(InterfaceList &interfaces) {
  RCLCPP_INFO(this->get_logger(), "Checking required interfaces...");

  bool required_interfaces_available = false;

  for (int i = 0; i < 10 && !required_interfaces_available; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for interfaces to initialize

    if(interfaces.arm && !srv_arm_->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Arming service not available.");
      continue;
    }
    if(interfaces.land && !srv_land_->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Landing service not available.");
      continue;
    }
    if(interfaces.takeoff && !act_takeoff_->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Takeoff action server not available.");
      continue;
    }
    if(interfaces.go_to_pos && !act_goToPos_->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "GoToPos action server not available.");
      continue;
    }
    //if(interfaces.set_gripper && !srv_set_gripper_->service_is_ready()) {
      //RCLCPP_ERROR(this->get_logger(), "SetGripper service not available. Service exists: %i Service is ready: %i ", interfaces.set_gripper, !srv_set_gripper_->service_is_ready());
      //continue;
    //}
    //if(interfaces.object_pose && 
      //  (sub_object_pose_->get_publisher_count() == 0 || 
       // !isValidPosition(object_telemetry_->getPosition())))
    //{
      //RCLCPP_ERROR(this->get_logger(), "Object pose topic not available.");
      //auto position = object_telemetry_->getPosition();
      //RCLCPP_INFO(this->get_logger(), "Object position: %f, %f, %f", position[0], position[1], position[2]);
      //continue;
    //}

    required_interfaces_available = true;
  }

  if (!required_interfaces_available) {
    RCLCPP_ERROR(this->get_logger(), "Not all required interfaces available.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "All required interfaces available.");
  return true;
}


void MissionControl::shutdown() {
  RCLCPP_ERROR(this->get_logger(), "Mission interrupted. Shutting down");
  rclcpp::shutdown();
  exit(0);
}


void MissionControl::objectPoseCallback(const PoseStamped::SharedPtr msg) {
  if (!update_object_pose_) {return;}
  // check vicon data (vicon will stream 0 fields if object cannot be tracked.)
  // if(!isValidPosition(position)) {
  //   RCLCPP_WARN(this->get_logger(), "Vicon object pose invalid.");
  //   return;
  // }

  // convert enu (vicon) to nwu (mission control) frame:
  // std::array<float, 3> position = {(float)msg->pose.position.y, 
  //                                   (float)-msg->pose.position.x, 
  //                                   (float)msg->pose.position.z};
  // convert from ned to nwu frame
  // std::array<float, 3> position = {(float)msg->pose.position.x, 
  //                                   (float)-msg->pose.position.y, 
  //                                   (float)-msg->pose.position.z};
  // with full object detection: target is already in NWU
  std::array<float, 3> position = {(float)msg->pose.position.x,
                                    (float)msg->pose.position.y,
                                    (float)msg->pose.position.z};

  current_object_pose_ = msg;
  object_telemetry_->setPosition(position);
  object_telemetry_count_++;

  // TODO attitude information?
}

void MissionControl::dronePoseCallback(const PoseStamped::SharedPtr msg) {
  current_drone_pose_ = msg;
}


bool MissionControl::isValidPosition(const std::array<float,3>& position, float threshold) {
  if(std::abs(position[0]) < threshold &&
     std::abs(position[1]) < threshold &&
     std::abs(position[2]) < threshold) {
    return false;
  }
  return true;
}