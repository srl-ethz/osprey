#include "rclcpp/rclcpp.hpp"
#include "mission_control/mission_control.hpp"


const int min_object_detection_count = 3;
const int object_closeup_count = 3;
const float object_closeup_rate = 2; // Hz
rclcpp::Rate object_closeup_rate_controller(object_closeup_rate);
const float object_closeup_timeout = 5; // s
const int object_closeup_iterations = (int)(object_closeup_timeout*object_closeup_rate + 0.5);


// define positions in mission's NWU frame
// assume drone starts at bottom left (towards tables)
// facing forward (i.e. VIO frame should align with global NWU frame)
// and initializes VIO-world frame there
std::array<float, 3> line_init = {0.0, -0.5, 0.3};
std::array<float, 3> line_start = {0.0, -0.5, 0.3};
std::array<float, 3> object_detection_offset = {-1.5, 0.0, 0.7};
const float line_increment = 0.5;
const float line_length = 3.0; // m
const int max_line_points = (int)(line_length/line_increment + 0.5);


std::array<float, 3> object_approach_offset = {0.0, 0.0, 0.3};
std::array<float, 3> object_grasp_offset = {0.0, 0.0, 0.18};
std::array<float, 3> object_leave_offset = object_approach_offset;
std::array<float, 3> object_drop = {1.0, 0.0, 0.3};
std::array<float, 3> object_land = {1.0, -1.0, 0.3};

void land_and_terminate(std::shared_ptr<MissionControl> mission_control_node) {
  mission_control_node->setGripper(70, 90); // open gripper
  if(!mission_control_node->land()) {mission_control_node->shutdown();}
  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");
  rclcpp::shutdown();
}


int main(int argc, char *argv[])
{
  // set required interfaces
  InterfaceList required_interfaces;
  required_interfaces.arm = true;
  required_interfaces.takeoff = true;
  required_interfaces.land = true;
  required_interfaces.go_to_pos = true;
  required_interfaces.set_gripper = true;
  required_interfaces.object_pose = false; // since the publisher only becomes alive upon detection

  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>(required_interfaces);
  if(!rclcpp::ok()) {mission_control_node->shutdown();}
  RCLCPP_INFO(mission_control_node->get_logger(), "Initialization complete. Starting mission.");

  // takeoff
  if(!mission_control_node->arm()) {mission_control_node->shutdown();}
  if(!mission_control_node->takeoff(0.7)) {mission_control_node->shutdown();}
  mission_control_node->setGripper(70, 90); // move front gripper out of camera view

  // mission
  if(!mission_control_node->go_to_pos(line_init, 0.0, 4.0, true)){mission_control_node->shutdown();}
  // search object - fly line
  bool found_object = false;
  for (int i = 0; i <= max_line_points; i++) {
    mission_control_node->object_telemetry_count_ = 0;
    if(!mission_control_node->go_to_pos({line_start[0] + i*line_increment, line_start[1], line_start[2]}, 0.0, 4.0, true)){mission_control_node->shutdown();}
    // check if object was detected
    if(mission_control_node->object_telemetry_count_ >= min_object_detection_count) {
      RCLCPP_INFO(mission_control_node->get_logger(), "Object detected.");
      found_object = true;
      break;
    }
    RCLCPP_INFO(mission_control_node->get_logger(), "Object samples after iteration: %d.", 
      mission_control_node->object_telemetry_count_);
  }
  if(!found_object) {
    RCLCPP_ERROR(mission_control_node->get_logger(), "Object not found - land now.");
    land_and_terminate(mission_control_node);
    return 0;
  }

  // get closeup object pose estimation
  if(!mission_control_node->go_to_object(object_detection_offset, 0.0, 4.0, true)){mission_control_node->shutdown();}
  mission_control_node->object_telemetry_count_ = 0;
  // get samples
  bool got_closeup_samples = false;
  for (int i = 0; i < object_closeup_iterations; i++) {
    rclcpp::spin_some(mission_control_node->get_node_base_interface());
    if (mission_control_node->object_telemetry_count_ >= object_closeup_count) {
      mission_control_node->update_object_pose_ = false; // stop updating object pose
      RCLCPP_INFO(mission_control_node->get_logger(), "Got sufficient closeup object samples - continue to grasp.");
      got_closeup_samples = true;
      break;
    }
    object_closeup_rate_controller.sleep();
  }
  mission_control_node->update_object_pose_ = false; // stop updating object pose
  // if(!got_closeup_samples) {
  //   RCLCPP_ERROR(mission_control_node->get_logger(), "Insufficient closeup samples after timeout - land now.");
  //   land_and_terminate(mission_control_node);
  //   return 0;
  // }
  
  // grasp object
  if(!mission_control_node->go_to_object(object_approach_offset, 0.0, 4.0, true)){mission_control_node->shutdown();}
  if(!mission_control_node->go_to_object(object_grasp_offset, 0.0, 3.0, true)){mission_control_node->shutdown();}
  mission_control_node->setGripper(0, 0); // close gripper
  if(!mission_control_node->go_to_object(object_leave_offset, 0.0, 2.0, true)){mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(object_drop, 0.0, 2.0, true)){mission_control_node->shutdown();}
  mission_control_node->setGripper(70, 90); // open gripper
  std::this_thread::sleep_for(std::chrono::milliseconds(250)); // wait a bit :)

  // land
  if(!mission_control_node->go_to_pos(object_land, 0.0, 1.8, true)) {mission_control_node->shutdown();}
  land_and_terminate(mission_control_node);
  return 0;
}
