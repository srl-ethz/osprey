// ros
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;


class VioPX4Interface : public rclcpp::Node
{
private:
  // ros
  const std::string slam_raw_topic_ = "/slam/odometry";
  const std::string slam_ned_topic_ = "/slam/pose_drone_ned";
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_odometry_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  /**
   * @brief Convert a rotation from spectacular's convention (C->W_nwu) to ned (B_ned->W_ned).
   * 
   * @param q_nwu_C: Quaternion representing the rotation in camera convention.
   * @param q_ned_nwu: Quaternion representing the rotation world nwu->ned
   * @param q_C_ned: Quaternion representing the rotation body-ned->C
   * 
   * @return Quaternion representing the rotation in ned convention.
  */
  tf2::Quaternion convert_rot_camera_to_ned(tf2::Quaternion q_nwu_C,
                                            tf2::Quaternion q_ned_nwu,
                                            tf2::Quaternion q_C_ned)
  {
    // convert rotation
    tf2::Quaternion q_ned = q_ned_nwu * q_nwu_C * q_C_ned;
    q_ned.normalize();
    return q_ned;
  }

  /**
   * @brief Transform a pose from spectacular's camera convention (C->W_NWU) to ned (B_NED->W_NED).
   * 
   * @throws tf2::TransformException if the required tf2 transforms are unavailable.
  */
  geometry_msgs::msg::PoseStamped transform_camera_to_ned(geometry_msgs::msg::PoseStamped pose_in) 
  {
    geometry_msgs::msg::PoseStamped pose_out = pose_in;
    pose_out.header.frame_id = "world_ned";

    // get the required tf2 transforms
    // Require: W_NWU->W_NED and C->D_NED
    geometry_msgs::msg::TransformStamped t_ned_nwu = 
      tf_buffer_->lookupTransform("world_ned", "world_nwu", rclcpp::Time(0));
    geometry_msgs::msg::TransformStamped t_ned_C =
      tf_buffer_->lookupTransform("body_ned", "cam_spec", rclcpp::Time(0));
    
    tf2::Quaternion q_ned_nwu, q_ned_C;
    tf2::fromMsg(t_ned_nwu.transform.rotation, q_ned_nwu);
    tf2::fromMsg(t_ned_C.transform.rotation, q_ned_C);

    // transform rotation
    tf2::Quaternion q_Wned_Bned;
    tf2::fromMsg(pose_in.pose.orientation, q_Wned_Bned);
    q_Wned_Bned = convert_rot_camera_to_ned(q_Wned_Bned, q_ned_nwu, q_ned_C.inverse());
    pose_out.pose.orientation = tf2::toMsg(q_Wned_Bned);

    // transform translation from NWU to NED (flip y and z axes)
    pose_out.pose.position.x = pose_in.pose.position.x;
    pose_out.pose.position.y = -pose_in.pose.position.y;
    pose_out.pose.position.z = -pose_in.pose.position.z;
    // add translation offsets from camera to body frame (in W-NED frame)
    // static offset is given in B_ned, converted rotation is q_Wned_Bned
    // we can use this rotation to rotate the offset from B_ned to W_ned
    tf2::Vector3 offset_B_ned(t_ned_C.transform.translation.x,
                              t_ned_C.transform.translation.y,
                              t_ned_C.transform.translation.z);
    tf2::Vector3 offset_W_ned = tf2::quatRotate(q_Wned_Bned, offset_B_ned);
    pose_out.pose.position.x -= offset_W_ned.x();
    pose_out.pose.position.y -= offset_W_ned.y();
    pose_out.pose.position.z -= offset_W_ned.z();

    return pose_out;
  }

  void odometry_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
  {
    // TODO check data
    // What is the failure mechanism of spectacularVIO?

    geometry_msgs::msg::PoseStamped pose;
    try { // catch transform exception here
      pose = transform_camera_to_ned(*msg.get()); 
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return; // don't publish anything
    }

    pub_pose_->publish(pose);

    // DEBUG
    // RCLCPP_INFO(this->get_logger(), 
    //             "VIO->PX4: roll: %f, pitch: %f, yaw: %f",
    //             vision_msg.angle_body.roll_rad * 180 / M_PI,
    //             vision_msg.angle_body.pitch_rad * 180 / M_PI,
    //             vision_msg.angle_body.yaw_rad * 180 / M_PI);
  }

public:
  VioPX4Interface(std::shared_ptr<Telemetry> telemetry, std::shared_ptr<Mocap> vision)
  : Node("vio_publisher"), telemetry_(telemetry), vision_(vision)
  {
    sub_odometry_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      slam_raw_topic_, 10, std::bind(&VioPX4Interface::odometry_callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(slam_ned_topic_, 10);
  }
};


int main(int argc, char *argv[]) 
{
  // init ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VioPX4Interface>(telemetry, vision);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "VioPublisher ready subscribing to %s.",
      slam_raw_topic_.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
}
