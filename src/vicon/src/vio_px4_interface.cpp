// ros
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// mavsdk
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


class VioPX4Interface : public rclcpp::Node
{
private:
  // ros
  const std::string slam_raw_topic_ = "/slam/odometry";
  const std::string slam_ned_topic_ = "/slam/pose_drone_ned";
  const std::string px4_pose_topic_ = "/px4/pose/ned";
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_px4_pose_;
  rclcpp::TimerBase::SharedPtr timer_px4_pub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // mavsdk
  std::shared_ptr<Telemetry> telemetry_;
  std::shared_ptr<Mocap> vision_;
  // std::shared_ptr<System> system;

  // pose transformations - TODO get this transform from tf2
  // translation from camera to body frame, expressed in body frame and meters
  // const double x_offset_DC_m = -0.08;
  // const double y_offset_DC_m = 0.0;
  // const double z_offset_DC_m = 0.02;
  // TODO this might have to be inverted (rotation from camera to drone frame)
  // camera points down 40 degrees and uses frame: z front, y up, x left
  // double roll_offset_DC = 50.0 * M_PI / 180.0;
  // double pitch_offset_DC = 0.0 * M_PI / 180.0;
  // double yaw_offset_DC = 90.0 * M_PI / 180.0;

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

    Mocap::VisionPositionEstimate vision_msg = convert_to_vision_estimate(pose);
    vision_->set_vision_position_estimate(vision_msg);
    pose_publisher_->publish(pose);

    // DEBUG
    RCLCPP_INFO(this->get_logger(), 
                "VIO->PX4: roll: %f, pitch: %f, yaw: %f",
                vision_msg.angle_body.roll_rad * 180 / M_PI,
                vision_msg.angle_body.pitch_rad * 180 / M_PI,
                vision_msg.angle_body.yaw_rad * 180 / M_PI);
  }

public:
  VioPX4Interface(std::shared_ptr<Telemetry> telemetry, std::shared_ptr<Mocap> vision)
  : Node("vio_px4_interface"), telemetry_(telemetry), vision_(vision)
  {
    odometry_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      slam_raw_topic_, 10, std::bind(&VioPX4Interface::odometry_callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(slam_ned_topic_, 10);
    pub_px4_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(px4_pose_topic_, 10);
    // timers
    timer_px4_pub_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                         std::bind(&VioPX4Interface::stream_px4_pose, this));
  }
};






// MavSDK helper

const int INTERVAL_RATE_HZ = 100;
const int INTERVAL_MS = 1000. / INTERVAL_RATE_HZ;
const int TELEMETRY_RATE_HZ = 100;

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
  // TIDI get port as commandline argument
  Mavsdk mavsdk;
  ConnectionResult connection_result =
    // Max Xbee
    // mavsdk.add_any_connection("serial:///dev/tty.usbserial-D309S1F2");
    // Mac usb port
    // mavsdk.add_any_connection("serial:///dev/tty.usbmodem01");
    // Raspberry pi usb port
    mavsdk.add_any_connection("serial:///dev/ttyACM0");
    // Raspberry pi FTDI
    // mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success) {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }
  auto system = get_system(mavsdk);
  if (!system) {
    return 1;
  }

  // Instantiate plugins
  auto telemetry = std::make_shared<Telemetry>(system);
  // auto vision = Mocap{system};
  auto vision = std::make_shared<Mocap>(system);
  auto mavlink = MavlinkPassthrough{system};

  // set telemetry subscription rate
  // auto sub_rate_result = telemetry.set_rate_position_velocity_ned(TELEMETRY_RATE_HZ);
  // if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
  // std::cout << "Failed to set subscription rate for pos_vel: " << (int)sub_rate_result << std::endl;
  //     return 1;
  // }
  // sub_rate_result = telemetry.set_rate_attitude(TELEMETRY_RATE_HZ);
  // if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
  // std::cout << "Failed to set subscription rate for attitude: " << (int)sub_rate_result << std::endl;
  //     return 1;
  // }
  // sub_rate_result = telemetry.set_rate_imu(TELEMETRY_RATE_HZ);
  // if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
  // std::cout << "Failed to set subscription rate for imu: " << (int)sub_rate_result << std::endl;
  //     return 1;
  // }
  // std::cout << "Successfully set subscription rates for pos_vel, attitude, and imu." << std::endl;

    // init ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VioPX4Interface>(telemetry, vision);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "VioPX4Interface ready subscribing to /slam/odometry.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
