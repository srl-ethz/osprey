#include "mission_control/mission_control.hpp"




bool MissionControl::land() {
  // TODO check quad state

  // check if service is available TODO
  if (!srv_land_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Land service not found.");
    return false;
  }

  auto request = std::make_shared<Trigger::Request>(); // create request
  auto result = srv_land_->async_send_request(request); // send request


  // wait until service completed
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::seconds(3)) 
      == rclcpp::FutureReturnCode::SUCCESS) {
      
    // get response from future
    auto response = result.get();

    // check response
    if (response->result) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to land: [%i]", response->result);
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quad is landing.");
    return true; // success

  } else { // service call unsuccessfull
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Land service failed (ROS error).");
    return false;
  }
}