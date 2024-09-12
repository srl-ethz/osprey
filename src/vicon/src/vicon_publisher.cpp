// ros
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


class ViconPX4Interface : public rclcpp::Node
{
private:
  // ros
  const std::string vicon_raw_topic_ = "/vicon/srl_osprey/srl_osprey";
  const std::string vicon_ned_topic_ = "/vicon/srl_osprey/ned";
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vicon_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_vicon_ned_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief Convert a rotation expressed in enu (B_enu->W_enu) to ned (B_ned->W_ned).
   * 
   * @param q_enu: Quaternion representing the rotation in enu convention.
   * 
   * @return Quaternion representing the rotation in ned convention.
   * @throws tf2::TransformException if the transform from world to world_ned frame is not available.
  */
  tf2::Quaternion convert_rot_enu_to_ned(tf2::Quaternion q_enu) 
  {
    tf2::Quaternion q_ned;

    // Get the static transform world_enu->world_ned frame
    // Note: This will throw a tf2::TransformException if the transform is not available.
    geometry_msgs::msg::TransformStamped transformStamped = 
      tf_buffer_->lookupTransform("world_ned", "world", rclcpp::Time(0));
    // extract the quaternion from the transform
    tf2::Quaternion q_ned_enu;
    tf2::fromMsg(transformStamped.transform.rotation, q_ned_enu);

    // convert the rotation
    q_ned = q_ned_enu * q_enu * q_ned_enu.inverse();
    q_ned.normalize();

    // DEBUG
    // // get rpy from quaternion
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q_ned_enu).getRPY(roll, pitch, yaw);
    // // convert to degrees
    // RCLCPP_INFO(this->get_logger(), 
    //         "W2NED: roll: %f, pitch: %f, yaw: %f",
    //             roll * 180 / M_PI,
    //             pitch * 180 / M_PI,
    //             yaw * 180 / M_PI);

    return q_ned;
}

  /**
   * @brief Transform a pose given in enu convention (vicon) to ned convention.
   * 
   * Given a pose in body enu convention (B_enu->W_enu), 
   * transforms it to body ned convention (B_ned->W_ned).
   * 
   * @throws tf2::TransformException if the transform from world to world_ned frame is not available.
  */
  geometry_msgs::msg::PoseStamped transform_vicon_to_ned(geometry_msgs::msg::PoseStamped pose_in) 
  {
    geometry_msgs::msg::PoseStamped pose_out = pose_in;
    pose_out.header.frame_id = "world_ned";
    
    // TODO get tf2 transform here

    // transform translation
    pose_out.pose.position.x = pose_in.pose.position.y;
    pose_out.pose.position.y = pose_in.pose.position.x;
    pose_out.pose.position.z = -pose_in.pose.position.z;
    // transform rotation
    tf2::Quaternion q;
    tf2::fromMsg(pose_in.pose.orientation, q);
    q = convert_rot_enu_to_ned(q);
    pose_out.pose.orientation = tf2::toMsg(q);

    return pose_out;
  }


  void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
  {
    // TODO check data
    // Vicon will stream 0 fields if object cannot be tracked.

    // The EKF2 on the PX4 only supports two MAVLink messages:
    // VISION_POSITION_ESTIMATE and ODOMETRY(frame_id=MAV_FRAME_LOCAL_FRD) 
    // Specifically, ATT_POS_MOCAP is not supported.
    // The ODOMETRY message offers quaternion (and generally a richer) support.
    // See https://docs.px4.io/v1.14/en/ros/external_position_estimation.html,
    // https://mavlink.io/en/messages/common.html#MAV_FRAME

    geometry_msgs::msg::PoseStamped pose;
    try { // catch transform exception here
      pose = transform_vicon_to_ned(*msg.get()); 
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return; // don't publish anything
    }

    pub_vicon_ned_->publish(pose);
  }

public:
  ViconPX4Interface() 
  : Node("vicon_px4_interface")
  {

    // susbcriptions
    sub_vicon_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      vicon_raw_topic_, 10, std::bind(&ViconPX4Interface::vicon_callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // publications
    pub_vicon_ned_ = 
      this->create_publisher<geometry_msgs::msg::PoseStamped>(vicon_ned_topic_, 10);
  }
};



int main(int argc, char *argv[]) 
{
  // init ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViconPX4Interface>();
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
  //     "ViconPublisher ready subscribing to %s.",
  //     vicon_raw_topic_.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
}