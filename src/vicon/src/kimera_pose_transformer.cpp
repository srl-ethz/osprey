#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav_msgs/msg/odometry.hpp" // TODO
// #include "geometry_msgs/msg/pose.hpp"
#include "osprey_interface/msg/pose.hpp"


class PoseTransformer : public rclcpp::Node
{
public:
  PoseTransformer() : Node("pose_transformer_node")
  {
    // create subscirber to std_msgs::msg::Pose 
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/transformed_odom", 10, std::bind(&PoseTransformer::poseCallback, this, std::placeholders::_1));
    // create publisher
    pub_ = this->create_publisher<osprey_interface::msg::Pose>("kimera_pose_nwu", 10);
  }

private:
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr received_msg)
  {
    auto pose = received_msg->pose.pose;

    osprey_interface::msg::Pose outgoing_msg;
    outgoing_msg.x_m = pose.position.x;
    outgoing_msg.y_m = pose.position.y;
    outgoing_msg.z_m = pose.position.z;


    // convert the incoming quaternion to roll, pitch, yaw
    tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
    tf2::Matrix3x3 m(q); 

    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    outgoing_msg.roll_deg = roll * 180.0 / M_PI;
    outgoing_msg.pitch_deg = pitch * 180.0 / M_PI;
    outgoing_msg.yaw_deg = yaw * 180.0 / M_PI;

    RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", outgoing_msg.roll_deg, outgoing_msg.pitch_deg, outgoing_msg.yaw_deg);

    // publish message
    pub_->publish(outgoing_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<osprey_interface::msg::Pose>::SharedPtr pub_;
};




int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize the node
  auto transformer_node = std::make_shared<PoseTransformer>();

  // info msg that node is running
  RCLCPP_INFO(transformer_node->get_logger(), "Optimus Prime ready for action.");
  // run the node until ros is not ok
  rclcpp::spin(transformer_node);

  rclcpp::shutdown();
  return 0;
}