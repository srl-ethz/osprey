////////////////////////////////////////////////////////////////////////////////
// ROS

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h" // TODO
#include "tf2/LinearMath/Matrix3x3.h" // TODO

#include "nav_msgs/msg/odometry.hpp" // TODO

#include "osprey_interface/msg/pose.hpp"
#include "osprey_interface/msg/velocity.hpp"
#include "osprey_interface/msg/acceleration.hpp"




////////////////////////////////////////////////////////////////////////////////
// MavSDK, PX4

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "PX4-Matrix/matrix/math.hpp"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;






// parameters

const std::string KIMERA_POSE_TOPIC = "/transformed_odom"; // TODO









////////////////////////////////////////////////////////////////////////////////
// MavSDK helper

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
  if (fut.wait_for(seconds(3)) == std::future_status::timeout)
  {
    std::cerr << "No autopilot found.\n";
    return {};
  }

  // Get discovered system now.
  return fut.get();
}




////////////////////////////////////////////////////////////////////////////////
// ROS publisher

class PX4Transmitter : public rclcpp::Node
{
public:
  PX4Transmitter() : Node("px4_kimera_transmitter")
  {
    // init ros interface to kimear
    sub_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(KIMERA_POSE_TOPIC, 10, std::bind(&PX4Transmitter::pose_callback, this, std::placeholders::_1));

    // init ros interface for publishing
    pose_publisher_ = this->create_publisher<osprey_interface::msg::Pose>("px4_pose_nwu", 10);
    velocity_publisher_ = this->create_publisher<osprey_interface::msg::Velocity>("px4_vel_nwu", 10);
    acceleration_publisher_ = this->create_publisher<osprey_interface::msg::Acceleration>("px4_acc_flu", 10);

    // init vision msg
    vision_msg_.time_usec = 0;
    std::vector<float> v = {NAN};
    vision_msg_.pose_covariance.covariance_matrix = v;
  }

  void publish_pose(osprey_interface::msg::Pose &message) 
  {
    pose_publisher_->publish(message);
  }

  void publish_velocity(osprey_interface::msg::Velocity &message) 
  {
    velocity_publisher_->publish(message);
  }

  void publish_acceleration(osprey_interface::msg::Acceleration &message) 
  {
    acceleration_publisher_->publish(message);
  }

private:
  // ros interface for subscribing to kimera
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ros interface for publishing the px4's estimates
  rclcpp::Publisher<osprey_interface::msg::Pose>::SharedPtr pose_publisher_;
  rclcpp::Publisher<osprey_interface::msg::Velocity>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<osprey_interface::msg::Acceleration>::SharedPtr acceleration_publisher_;

  // mavsdk
  std::shared_ptr<Telemetry> telemetry_;
  std::shared_ptr<Mocap> vision_;
  Mocap::VisionPositionEstimate vision_msg_;
};

void PX4Transmitter::pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto pose = msg->pose.pose;
  auto q_msg = pose.orientation; // float x, y, z, w 

  // transform this quaternion to euler angles
  tf2::Quaternion q(
    q_msg.x,
    q_msg.y,
    q_msg.z,
    q_msg.w);
  tf2::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);



  // TODO set vision estimate
  vision_msg_.position_body.x_m = pose.position.x;
  vision_msg_.position_body.y_m = pose.position.y;
  vision_msg_.position_body.z_m = pose.position.z;
  vision_msg_.angle_body.roll_rad =
  vision_msg_.angle_body.pitch_rad =
  vision_msg_.angle_body.yaw_rad = 

  vision_.set_vision_position_estimate(vision_msg_);
}

////////////////////////////////////////////////////////////////////////////////
// main

int main(int argc, char *argv[])
{
  // check command line argument
  if (argc < 2)
  {
    usage(argv[0]);
    return 1;
  }

  ////////////////////////////////////////////////////////////////// init mavsdk
  // Max Xbee
  // mavsdk.add_any_connection("serial:///dev/tty.usbserial-D309S1F2");
  // Mac usb port
  // mavsdk.add_any_connection("serial:///dev/tty.usbmodem01");
  // Raspberry pi usb port
  // mavsdk.add_any_connection("serial:///dev/ttyACM0");
  // Raspberry pi FTDI
  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success) {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }
  auto system = get_system(mavsdk);
  if (!system) {
    return 1;
  }

  // Instantiate plugins
  auto telemetry = Telemetry{system};
  auto vision = Mocap{system};
  auto mavlink = MavlinkPassthrough{system};

  // set telemetry subscription rate
  auto sub_rate_result = telemetry.set_rate_position_velocity_ned(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
   std::cout << "Failed to set subscription rate for pos_vel: " << (int)sub_rate_result << std::endl;
    return 1;
  }
  sub_rate_result = telemetry.set_rate_attitude(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
   std::cout << "Failed to set subscription rate for attitude: " << (int)sub_rate_result << std::endl;
    return 1;
  }
  sub_rate_result = telemetry.set_rate_imu(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
   std::cout << "Failed to set subscription rate for imu: " << (int)sub_rate_result << std::endl;
    return 1;
  }
  std::cout << "Successfully set subscription rates for pos_vel, attitude, and imu." << std::endl;



  ///////////////////////////////////////////////////////////////////// init ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PX4Transmitter>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kimera to PX4 Transmitter initialized.");



  // TODO publish px4 telemetry data
  // telemetry
  auto pose_msg = osprey_interface::msg::Pose();
  auto vel_msg = osprey_interface::msg::Velocity();
  // auto acc_msg = osprey_interface::msg::Acceleration();

  const auto& pos_vel = telemetry.position_velocity_ned();
  const auto& attitude_euler = telemetry.attitude_euler();
  // const auto& imu = telemetry.imu();
  
  pose_msg.x_m = pos_vel.position.north_m;
  pose_msg.y_m = - pos_vel.position.east_m;
  pose_msg.z_m = - pos_vel.position.down_m;
  pose_msg.roll_deg = attitude_euler.roll_deg;
  pose_msg.pitch_deg = -attitude_euler.pitch_deg;
  pose_msg.yaw_deg = -attitude_euler.yaw_deg;

  vel_msg.x_m_s = pos_vel.velocity.north_m_s;
  vel_msg.y_m_s = - pos_vel.velocity.east_m_s;
  vel_msg.z_m_s = - pos_vel.velocity.down_m_s;

  // acc_msg.x_m_s2 = imu.acceleration_frd.forward_m_s2;
  // acc_msg.y_m_s2 = - imu.acceleration_frd.right_m_s2;
  // acc_msg.z_m_s2 = - imu.acceleration_frd.down_m_s2;

  node->publish_pose(pose_msg);
  node->publish_velocity(vel_msg);
  // node->publish_acceleration(acc_msg);











  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
