#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "osprey_interface/msg/pose.hpp"


class PoseTransformer : public rclcpp::Node
{
public:
  PoseTransformer() : Node("pose_transformer_node")
  {
    // create subscirber
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/slam/odometry", 10, std::bind(&PoseTransformer::poseCallback, this, std::placeholders::_1));
    // create publisher
    pub_ = this->create_publisher<osprey_interface::msg::Pose>("vo_pose_nwu", 10);
  }

private:
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr received_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received.");
    osprey_interface::msg::Pose outgoing_msg;
    
    outgoing_msg.x_m = received_msg->pose.pose.position.x;
    outgoing_msg.y_m = received_msg->pose.pose.position.y;
    outgoing_msg.z_m = received_msg->pose.pose.position.z;


    // convert the incoming quaternion to roll, pitch, yaw
    tf2::Quaternion q(
      received_msg->pose.pose.orientation.x,
      received_msg->pose.pose.orientation.y,
      received_msg->pose.pose.orientation.z,
      received_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q); 

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    outgoing_msg.roll_deg = roll * 180.0 / M_PI;
    outgoing_msg.pitch_deg = pitch * 180.0 / M_PI;
    outgoing_msg.yaw_deg = yaw * 180.0 / M_PI;

    // publish message
    pub_->publish(outgoing_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<osprey_interface::msg::Pose>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};




int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize the node
  auto transformer_node = std::make_shared<PoseTransformer>();

  // info msg that node is running
  RCLCPP_INFO(transformer_node->get_logger(), "Ready.");
  // run the node until ros is not ok
  rclcpp::spin(transformer_node);

  rclcpp::shutdown();
  return 0;
}