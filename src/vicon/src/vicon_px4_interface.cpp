// ros
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// mavsdk
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "PX4-Matrix/matrix/math.hpp"

using namespace mavsdk;


class ViconPX4Interface : public rclcpp::Node
{
private:
  // ros
  const std::string vicon_raw_topic_ = "/vicon/srl_osprey/srl_osprey";
  const std::string vicon_ned_topic_ = "/vicon/srl_osprey/ned";
  const std::string px4_pose_topic_ = "/px4/pose/ned";
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vicon_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_vicon_ned_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_px4_pose_;
  rclcpp::TimerBase::SharedPtr timer_px4_pub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // mavsdk
  std::shared_ptr<Telemetry> telemetry_;
  std::shared_ptr<Mocap> vision_;

  // pose transformation - unused since we get the transformation from tf2
  // const double roll_offset_ = M_PI;
  // const double pitch_offset_ = 0;
  // const double yaw_offset_ = -M_PI/2;

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

  /**
   * @brief Converts a PoseStamped message to a VisionPositionEstimate message.
  */
  Mocap::VisionPositionEstimate convert_to_vision_estimate(geometry_msgs::msg::PoseStamped pose) 
  {
    Mocap::VisionPositionEstimate vision_msg;

    // initalize message, without this, PX4 won't accept the message
    rclcpp::Time timestamp = pose.header.stamp;
    int64_t timestamp_microseconds = timestamp.nanoseconds() / 1000;
    vision_msg.time_usec = timestamp_microseconds;
    std::vector<float> v = {NAN};
    vision_msg.pose_covariance.covariance_matrix = v;

    // translation
    vision_msg.position_body.x_m = pose.pose.position.x;
    vision_msg.position_body.y_m = pose.pose.position.y;
    vision_msg.position_body.z_m = pose.pose.position.z;

    // extract quaternion from pose msg
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    // convert quaternion to euler angles using PX4-Matrix (tf2 uses different euler conventions)
    // TF2 vs PX4 quaternion conventions:
    // TF2 convention: x, y, z, w
    // PX4 convention: w, x, y, z
    matrix::Quatf px4_quat(q.w(), q.x(), q.y(), q.z());
    matrix::Eulerf euler_orientation(px4_quat);
    vision_msg.angle_body.roll_rad = euler_orientation(0);
    vision_msg.angle_body.pitch_rad = euler_orientation(1);
    vision_msg.angle_body.yaw_rad = euler_orientation(2);

    return vision_msg;
  }

  /**
   * @brief Stream the PX4 pose to ROS.
   * 
   * Access the PX4 pose estimates through the telemetry plugin and publish them to ROS.
  */
  void stream_px4_pose() 
  {
    // get px4 pose
    auto px4_pos_vel = telemetry_->position_velocity_ned();
    auto px4_attitude = telemetry_->attitude_euler();
    // assemble message
    geometry_msgs::msg::PoseStamped px4_pose_msg;
    px4_pose_msg.header.frame_id = "world_ned";
    px4_pose_msg.header.stamp = this->now(); // TODO can we use the PX4 timestamp?
    px4_pose_msg.pose.position.x = px4_pos_vel.position.north_m;
    px4_pose_msg.pose.position.y = px4_pos_vel.position.east_m;
    px4_pose_msg.pose.position.z = px4_pos_vel.position.down_m;
    tf2::Quaternion q;
    // TODO euler to quaternion using PX4-Matrix convention
    q.setRPY(px4_attitude.roll_deg * M_PI / 180,
              px4_attitude.pitch_deg * M_PI / 180,
              px4_attitude.yaw_deg * M_PI / 180);
    px4_pose_msg.pose.orientation = tf2::toMsg(q);
    // publish message
    pub_px4_pose_->publish(px4_pose_msg);
  }

  void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
  {
    // check vicon data (vicon will stream 0 fields if object cannot be tracked.)
    if (msg->pose.position.x < 1e-4 && 
        msg->pose.position.y < 1e-4 && 
        msg->pose.position.z < 1e-4) {
      RCLCPP_WARN(this->get_logger(), "Vicon pose invalid.");
      return;
    }

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

    Mocap::VisionPositionEstimate vision_msg = convert_to_vision_estimate(pose);
    vision_->set_vision_position_estimate(vision_msg);
    pub_vicon_ned_->publish(pose);

    // DEBUG
    RCLCPP_INFO(this->get_logger(), 
                "Vicon->PX4: roll: %f, pitch: %f, yaw: %f",
                vision_msg.angle_body.roll_rad * 180 / M_PI,
                vision_msg.angle_body.pitch_rad * 180 / M_PI,
                vision_msg.angle_body.yaw_rad * 180 / M_PI);
  }

public:
  ViconPX4Interface(std::shared_ptr<Telemetry> telemetry, std::shared_ptr<Mocap> vision) 
  : Node("vicon_px4_interface"), telemetry_(telemetry), vision_(vision) 
  {

    // susbcriptions
    sub_vicon_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      vicon_raw_topic_, 10, std::bind(&ViconPX4Interface::vicon_callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // publications
    pub_vicon_ned_ = 
      this->create_publisher<geometry_msgs::msg::PoseStamped>(vicon_ned_topic_, 10);
    pub_px4_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(px4_pose_topic_, 10);
    // timers
    timer_px4_pub_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                         std::bind(&ViconPX4Interface::stream_px4_pose, this));
  }
};



// mavsdk helpers

void usage(const std::string &bin_name)
{
  std::cerr
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk &mavsdk)
{
  std::cout << "Waiting to discover system...\n";
  auto prom = std::promise<std::shared_ptr<System>>{};
  auto fut = prom.get_future();

  // We wait for new systems to be discovered, once we find one that has an
  // autopilot, we decide to use it.
  mavsdk.subscribe_on_new_system([&mavsdk, &prom]()
                                 {
    auto system = mavsdk.systems().back();

    if (system->has_autopilot()) {
      std::cout << "Discovered autopilot\n";

      // Unsubscribe again as we only want to find one system.
      mavsdk.subscribe_on_new_system(nullptr);
      prom.set_value(system);
    } });

  // We usually receive heartbeats at 1Hz, therefore we should find a
  // system after around 3 seconds max, surely.
  if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
  {
    std::cerr << "No autopilot found.\n";
    return {};
  }

  // Get discovered system now.
  return fut.get();
}


int main(int argc, char *argv[]) 
{
  // check cl arguments
  if (argc < 2) {
    std::cout << "No command line argument given. Required: Serial port for PX4 (e.g. serial:///dev/ttyACM0).\n";
    return 1;
  }

  // establish mavsdk connection
  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success) {
      std::cerr << "Connection failed: " << connection_result << '\n';
      return 1;
  }

  auto system = get_system(mavsdk);
  if (!system) {return 1;}

  // instantiate plugins
  auto vision = std::make_shared<Mocap>(system);
  auto telemetry = std::make_shared<Telemetry>(system);
  // auto vision = Mocap{system};
  // auto mavlink = MavlinkPassthrough{system};

  // TODO set telemetry subscription rate for position
  // const int TELEMETRY_RATE_HZ = 100;
  // auto sub_rate_result = telemetry->set_rate_position_velocity_ned(TELEMETRY_RATE_HZ);
  // if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
  // std::cout << "Failed to set subscription rate for pos_vel: " << (int)sub_rate_result << std::endl;
  //     return 1;
  // }
  // sub_rate_result = telemetry->set_rate_attitude(TELEMETRY_RATE_HZ);
  // if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
  // std::cout << "Failed to set subscription rate for attitude: " << (int)sub_rate_result << std::endl;
  //     return 1;
  // }
  // sub_rate_result = telemetry->set_rate_imu(TELEMETRY_RATE_HZ);
  // if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
  // std::cout << "Failed to set subscription rate for imu: " << (int)sub_rate_result << std::endl;
  //     return 1;
  // }
  // std::cout << "Successfully set subscription rates for pos_vel, attitude, and imu." << std::endl;

  // init ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViconPX4Interface>(telemetry, vision);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "ViconPX4Interface ready subscribing to vicon/srl_osprey/srl_osprey.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}