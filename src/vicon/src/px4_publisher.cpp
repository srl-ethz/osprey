// ros
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// mavsdk
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "PX4-Matrix/matrix/math.hpp"


using namespace mavsdk;

class PX4Publisher : public rclcpp::Node
{
private:
  // ros
  const std::string px4_pose_topic_ = "/px4/pose/ned";
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_px4_pose_;
  rclcpp::TimerBase::SharedPtr timer_px4_pub_;

  // mavsdk
  std::shared_ptr<Telemetry> telemetry_;
  std::shared_ptr<Mocap> vision_;

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

public:
  ViconPX4Interface(std::shared_ptr<Telemetry> telemetry, std::shared_ptr<Mocap> vision) 
  : Node("vicon_px4_interface"), telemetry_(telemetry), vision_(vision) 
  {
    // publications
    pub_px4_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(px4_pose_topic_, 10);
    // timers
    timer_px4_pub_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                         std::bind(&ViconPX4Interface::stream_px4_pose, this));
  }
};


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
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PX4Publisher ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}