///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef WIN32
#include <conio.h>   // For _kbhit()
#include <cstdio>    // For getchar()
#include <windows.h> // For Sleep()
#else
#include <unistd.h> // For sleep()
#endif              // WIN32

#include <string.h>
#include <time.h>

////////////////////////////////////////////////////////////////////////////////
// ROS

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "osprey_interface/msg/pose.hpp"
#include "osprey_interface/msg/velocity.hpp"
#include "osprey_interface/msg/acceleration.hpp"

////////////////////////////////////////////////////////////////////////////////
// Vicon

#include "DataStreamClient.h"
// #include "vicon/vicon_helper.h"
using namespace ViconDataStreamSDK::CPP;

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
// parameters

const int LOOP_RATE_HZ = 150;
const int TELEMETRY_RATE_HZ = 100;
// const int INTERVAL_MS = 1000. / LOOP_RATE_HZ;
rclcpp::Rate loop_rate(LOOP_RATE_HZ);

const std::string VICON_IDENTIFIER = "srl_osprey"; // TODO ros param
constexpr static float yaw_offset_degrees = 0; // yaw offset local frame
constexpr static float yaw_offset_radians = yaw_offset_degrees * M_PI / 180;




////////////////////////////////////////////////////////////////////////////////
// ROS publisher

class PX4Publisher : public rclcpp::Node
{
public:
  PX4Publisher()
  : Node("px4_vicon_publisher")
  {
    vicon_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/vicon/pose_drone_ned", 10);
    px4_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/px4/pose_drone_ned", 10);
    // pose_publisher_ = this->create_publisher<osprey_interface::msg::Pose>("px4_pose_nwu", 10);
    velocity_publisher_ = this->create_publisher<osprey_interface::msg::Velocity>("px4_vel_nwu", 10);
    // acceleration_publisher_ = this->create_publisher<osprey_interface::msg::Acceleration>("px4_acc_flu", 10);
  }

  void publish_vicon_pose(geometry_msgs::msg::PoseStamped &message) 
  {
    vicon_pose_pub_->publish(message);
  }

  void publish_px4_pose(geometry_msgs::msg::PoseStamped &message) 
  {
    px4_pose_pub_->publish(message);
  }

  // void publish_pose(osprey_interface::msg::Pose &message) 
  // {
  //   pose_publisher_->publish(message);
  // }

  void publish_velocity(osprey_interface::msg::Velocity &message) 
  {
    velocity_publisher_->publish(message);
  }

  // void publish_acceleration(osprey_interface::msg::Acceleration &message) 
  // {
  //   acceleration_publisher_->publish(message);
  // }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr px4_pose_pub_;
  // rclcpp::Publisher<osprey_interface::msg::Pose>::SharedPtr pose_publisher_;
  rclcpp::Publisher<osprey_interface::msg::Velocity>::SharedPtr velocity_publisher_;
  // rclcpp::Publisher<osprey_interface::msg::Acceleration>::SharedPtr acceleration_publisher_;
};




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
  Mavsdk mavsdk;
  ConnectionResult connection_result =
      // Max Xbee
      // mavsdk.add_any_connection("serial:///dev/tty.usbserial-D309S1F2");
      // Mac usb port
      // mavsdk.add_any_connection("serial:///dev/tty.usbmodem01");
      // Raspberry pi usb port
      // mavsdk.add_any_connection("serial:///dev/ttyACM0");
      // Raspberry pi FTDI
      mavsdk.add_any_connection(argv[1]);

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
  // sub_rate_result = telemetry.set_rate_imu(TELEMETRY_RATE_HZ);
  // if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
  //  std::cout << "Failed to set subscription rate for imu: " << (int)sub_rate_result << std::endl;
  //   return 1;
  // }
  std::cout << "Successfully set subscription rates for pos_vel, attitude, and imu." << std::endl;

  // Prepare MavSDK mocap command
  Mocap::VisionPositionEstimate vision_msg;
  vision_msg.time_usec = 0;
  std::vector<float> v = {NAN};
  vision_msg.pose_covariance.covariance_matrix = v;


  ///////////////////////////////////////////////////////////////////// init ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PX4Publisher>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "Initializing Vicon-to-PX4 publisher for '%s' and PX4-telemetry publisher using the topics 'quad_px4_pose_nwu', 'quad_px4_vel_nwu' and 'quad_px4_acc_flu'.",
      VICON_IDENTIFIER.c_str());


  ///////////////////////////////////////////////////// Vicon Datastream Options
  std::vector<std::string> Hosts;
  Hosts.push_back("10.10.10.5"); // vicon address LEO C 6
  if (Hosts.empty())
  {
    Hosts.push_back("localhost:801");
  }

  std::vector<std::string> HapticOnList(0);
  unsigned int ClientBufferSize = 0;
  std::string AxisMapping = "ZUp";
  std::vector<std::string> FilteredSubjects;
  std::vector<std::string> LocalAdapters;

  // bool bQuiet = false;
  // std::ostream &OutputStream(bQuiet ? NullStream : std::cout);
  std::ostream &OutputStream(std::cout);

  bool First = true;
  std::string HostName;
  for (const auto &rHost : Hosts)
  {
    if (!First)
    {
      HostName += ";";
    }
    HostName += rHost;
    First = false;
  }

  // Make a new client
  ViconDataStreamSDK::CPP::Client DirectClient;

  bool bOptimizeWireless = false;
  if (bOptimizeWireless)
  {
    const Output_ConfigureWireless ConfigureWirelessResult =
        DirectClient.ConfigureWireless();

    if (ConfigureWirelessResult.Result != Result::Success)
    {
      std::cout << "Wireless Config: " << ConfigureWirelessResult.Error
                << std::endl;
    }
  }

  // POI
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "Connecting to '%s'...",
      HostName.c_str());

  while (!DirectClient.IsConnected().Connected)
  {
    // Direct connection
    const Output_Connect ConnectResult = DirectClient.Connect(HostName);
    const bool ok = (ConnectResult.Result == Result::Success);

    if (!ok)
    {
      std::cout << "Warning - connect failed... ";
      switch (ConnectResult.Result)
      {
      case Result::ClientAlreadyConnected:
        std::cout << "Client Already Connected" << std::endl;
        break;
      case Result::InvalidHostName:
        std::cout << "Invalid Host Name" << std::endl;
        break;
      case Result::ClientConnectionFailed:
        std::cout << "Client Connection Failed" << std::endl;
        break;
      default:
        std::cout << "Unrecognized Error: " << ConnectResult.Result
                  << std::endl;
        break;
      }
      return 1;
    }

#ifdef WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
    // }

    // Enable some different data types
    DirectClient.EnableSegmentData();

    DirectClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
    // }

    // Set the global up axis
    DirectClient.SetAxisMapping(Direction::Forward, Direction::Left,
                                Direction::Up); // Z-up

    if (AxisMapping == "YUp")
    {
      DirectClient.SetAxisMapping(Direction::Forward, Direction::Up,
                                  Direction::Right); // Y-up
    }
    else if (AxisMapping == "XUp")
    {
      DirectClient.SetAxisMapping(Direction::Up, Direction::Forward,
                                  Direction::Left); // Z-up
    }

    // Output_GetAxisMapping _Output_GetAxisMapping = DirectClient.GetAxisMapping();
    // std::cout << "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis)
    //           << " Y-" << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
    //           << Adapt(_Output_GetAxisMapping.ZAxis) << std::endl;

    // Discover the version number
    // Output_GetVersion _Output_GetVersion = DirectClient.GetVersion();
    // std::cout << "Version: " << _Output_GetVersion.Major << "."
    //           << _Output_GetVersion.Minor << "." << _Output_GetVersion.Point
    //           << "." << _Output_GetVersion.Revision << std::endl;

    if (ClientBufferSize > 0)
    {
      DirectClient.SetBufferSize(ClientBufferSize);
      std::cout << "Setting client buffer size to " << ClientBufferSize
                << std::endl;
    }

    bool bSubjectFilterApplied = false;

    ViconDataStreamSDK::CPP::Client &MyClient(DirectClient);




    ////////////////////////////////////////////////////////////// main loop POI
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vicon interface is ready.");

    // Loop until a key is pressed
#ifdef WIN32
    while (!Hit())
#else
    while (true)
#endif
    {
      // Get a frame
      // OutputStream << "Waiting for new frame...";
      while (MyClient.GetFrame().Result != Result::Success)
      {
        // Sleep a little so that we don't lumber the CPU with a busy poll
        // #ifdef WIN32
        //         Sleep(200);
        // #else
        //         sleep(1);
        // #endif

        // OutputStream << "No frame received from Vicon.";
        // OutputStream << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No frame received from Vicon.");
        if (!rclcpp::ok()) {
          rclcpp::shutdown();
          exit(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // We have to call this after the call to get frame, otherwise we don't
      // have any subject info to map the name to ids
      if (!bSubjectFilterApplied)
      {
        for (const auto &rSubject : FilteredSubjects)
        {
          Output_AddToSubjectFilter SubjectFilterResult =
              MyClient.AddToSubjectFilter(rSubject);
          bSubjectFilterApplied = bSubjectFilterApplied ||
                                  SubjectFilterResult.Result == Result::Success;
        }
      }

      // ///////////////////////////////////////////////////////////
      // Set frame number
      // ///////////////////////////////////////////////////////////

      // Get the frame number POI
      // Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      // Output_GetFrameRate Rate = MyClient.GetFrameRate();
      // OutputStream << "Frame rate: " << Rate.FrameRateHz << std::endl;

      // Show frame rates
      for (unsigned int FramerateIndex = 0;
           FramerateIndex < MyClient.GetFrameRateCount().Count;
           ++FramerateIndex)
      {
        std::string FramerateName =
            MyClient.GetFrameRateName(FramerateIndex).Name;
        // double FramerateValue = MyClient.GetFrameRateValue(FramerateName).Value;

        // OutputStream << FramerateName << ": " << FramerateValue << "Hz"
        //              << std::endl;
      }
      // OutputStream << std::endl;

      // Get the timecode
      // Output_GetTimecode _Output_GetTimecode = MyClient.GetTimecode();

      ///////////////////////////////////////////////////////////
      // Set latency
      ///////////////////////////////////////////////////////////

      // Get the latency
      // float latency = MyClient.GetLatencyTotal().Total;

      // for (unsigned int LatencySampleIndex = 0;
      //      LatencySampleIndex < MyClient.GetLatencySampleCount().Count;
      //      ++LatencySampleIndex) {
      //   std::string SampleName =
      //       MyClient.GetLatencySampleName(LatencySampleIndex).Name;
      //   double SampleValue =
      //   MyClient.GetLatencySampleValue(SampleName).Value;

      //   OutputStream << "  " << SampleName << " " << SampleValue << "s"
      //                << std::endl;
      // }

      // OutputStream << std::endl;

      // Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber =
      //     MyClient.GetHardwareFrameNumber();
      // OutputStream << "Hardware Frame Number: "
      //              << _Output_GetHardwareFrameNumber.HardwareFrameNumber
      //              << std::endl;

      ///////////////////////////////////////////////////////////
      // find requested subject POI
      ///////////////////////////////////////////////////////////

      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      // OutputStream << "Subjects (" << SubjectCount << "):" << std::endl;

      bool subjectFound = false;
      unsigned int SubjectIndex = 0;
      for (unsigned int i = 0; i < SubjectCount; ++i)
      {
        std::string SubjectName = MyClient.GetSubjectName(i).SubjectName;

        if (SubjectName.compare(VICON_IDENTIFIER) == 0)
        {
          SubjectIndex = i;
          subjectFound = true;
          break;
        }
      }
      // exit if subject wasn't found
      if (!subjectFound) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
            "Subject '%s' not found.", VICON_IDENTIFIER.c_str());
        return 1;
      }

      // OutputStream << "  Subject #" << SubjectIndex << std::endl;
      // Get the subject name
      std::string SubjectName =
          MyClient.GetSubjectName(SubjectIndex).SubjectName;
      // OutputStream << "    Name: " << SubjectName << std::endl;

      ///////////////////////////////////////////////////////////
      // segments
      ///////////////////////////////////////////////////////////

      // Get the root segment
      // std::string RootSegment =
      //     MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
      // OutputStream << "    Root Segment: " << RootSegment << std::endl;

      // Count the number of segments
      unsigned int SegmentCount =
          MyClient.GetSegmentCount(SubjectName).SegmentCount;
      // OutputStream << "    Segments (" << SegmentCount << "):" <<
      // std::endl;

      // loop over all segments 
      // TODO there should only be one segment for the quad anyway?
      for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount;
            ++SegmentIndex)
      {
        // OutputStream << "      Segment #" << SegmentIndex << std::endl;

        // Get the segment name
        std::string SegmentName =
            MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
        // OutputStream << "        Name: " << SegmentName << std::endl;

        // Get the global segment translation
        Output_GetSegmentGlobalTranslation
            _Output_GetSegmentGlobalTranslation =
                MyClient.GetSegmentGlobalTranslation(SubjectName,
                                                      SegmentName);

        ///////////////////////////////////////////////////////////
        // global translation POI
        ///////////////////////////////////////////////////////////

        vision_msg.position_body.x_m =
            _Output_GetSegmentGlobalTranslation.Translation[0] / 1000.0;
        vision_msg.position_body.y_m =
            -_Output_GetSegmentGlobalTranslation.Translation[1] / 1000.0;
        vision_msg.position_body.z_m =
            -_Output_GetSegmentGlobalTranslation.Translation[2] / 1000.0;

        ///////////////////////////////////////////////////////////
        // global rotation POI
        ///////////////////////////////////////////////////////////

        // Get the global segment rotation in quaternion co-ordinates
        Output_GetSegmentGlobalRotationQuaternion
            _Output_GetSegmentGlobalRotationQuaternion =
                MyClient.GetSegmentGlobalRotationQuaternion(SubjectName,
                                                            SegmentName);

        // Set global rotation quaternion POI
        matrix::Quatf quat_orientation(
            _Output_GetSegmentGlobalRotationQuaternion.Rotation[1],
            _Output_GetSegmentGlobalRotationQuaternion.Rotation[2],
            _Output_GetSegmentGlobalRotationQuaternion.Rotation[3],
            _Output_GetSegmentGlobalRotationQuaternion.Rotation[0]);

        matrix::Eulerf euler_orientation(quat_orientation);

        // Roll transformation
        if (euler_orientation(0) > 0)
          vision_msg.angle_body.roll_rad = M_PI - euler_orientation(0);

        else if (euler_orientation(0) < 0)
          vision_msg.angle_body.roll_rad = -M_PI - euler_orientation(0);

        // Pitch transformation
        vision_msg.angle_body.pitch_rad = -euler_orientation(1);

        if (euler_orientation(2) > 0)
          vision_msg.angle_body.yaw_rad = M_PI - euler_orientation(2);

        else if (euler_orientation(2) < 0)
          vision_msg.angle_body.yaw_rad = -M_PI - euler_orientation(2);

        // Invert Sign to match px4 convention
        vision_msg.angle_body.yaw_rad = -vision_msg.angle_body.yaw_rad;

        //   // Get the global segment rotation in EulerXYZ co-ordinates
        //   Output_GetSegmentGlobalRotationEulerXYZ
        //       _Output_GetSegmentGlobalRotationEulerXYZ =
        //           MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName,
        //                                                     SegmentName);
      }

      // // Get the quality of the subject (object) if supported
      // Output_GetObjectQuality _Output_GetObjectQuality =
      //     MyClient.GetObjectQuality(SubjectName);
      // if (_Output_GetObjectQuality.Result == Result::Success) {
      //   double Quality = _Output_GetObjectQuality.Quality;
      //   OutputStream << "    Quality: " << Quality << std::endl;
      // }




      /////////////////////////////////////////////////////////// POI
      // timer
      loop_time_ms = clock.now() - loop_start_time;
      // Run at requested interval
      std::this_thread::sleep_until(loop_start_time + std::chrono::milliseconds(INTERVAL_MS));
      // reset clock
      loop_start_time = clock.now();

      // relay data to px4 only if data is valid
      if (!(abs(vision_msg.position_body.x_m) < 0.0001 &&
            abs(vision_msg.position_body.y_m) < 0.0001 &&
            abs(vision_msg.position_body.z_m) < 0.0001) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vicon data is invalid.");
        continue;
      }

      vision.set_vision_position_estimate(vision_msg);

      tf2::Quaternion quat;

      // publish vicon pose
      auto vicon_pose_msg = geometry_msgs::msg::PoseStamped();
      vicon_pose_msg.header.stamp = node->now();
      vicon_pose_msg.pose.position.x = vision_msg.position_body.x_m;
      vicon_pose_msg.pose.position.y = vision_msg.position_body.y_m;
      vicon_pose_msg.pose.position.z = vision_msg.position_body.z_m;
      quat.setRPY(vision_msg.angle_body.roll_rad, 
                  vision_msg.angle_body.pitch_rad, 
                  vision_msg.angle_body.yaw_rad);
      vicon_pose_msg.pose.orientation = tf2::toMsg(quat);
      node->publish_vicon_pose(vicon_pose_msg);

      // publish px4 pose
      const auto& pos_vel = telemetry.position_velocity_ned();
      const auto& attitude_euler = telemetry.attitude_euler();

      auto px4_pose_msg = geometry_msgs::msg::PoseStamped();
      px4_pose_msg.header.stamp = node->now();
      px4_pose_msg.pose.position.x = pos_vel.position.north_m;
      px4_pose_msg.pose.position.y = pos_vel.position.east_m;
      px4_pose_msg.pose.position.z = pos_vel.position.down_m;
      quat.setRPY(attitude_euler.roll_deg * M_PI / 180, 
                  attitude_euler.pitch_deg * M_PI / 180,
                  attitude_euler.yaw_deg * M_PI / 180);
      px4_pose_msg.pose.orientation = tf2::toMsg(quat);
      node->publish_px4_pose(px4_pose_msg);

      // telemetry TODO convert to pose stamped
      const auto& imu = telemetry.imu();
      // auto pose_msg = osprey_interface::msg::Pose();
      auto vel_msg = osprey_interface::msg::Velocity();
      auto acc_msg = osprey_interface::msg::Acceleration();

      
      // pose_msg.x_m = pos_vel.position.north_m;
      // pose_msg.y_m = - pos_vel.position.east_m;
      // pose_msg.z_m = - pos_vel.position.down_m;
      // pose_msg.roll_deg = attitude_euler.roll_deg;
      // pose_msg.pitch_deg = -attitude_euler.pitch_deg;
      // pose_msg.yaw_deg = -attitude_euler.yaw_deg;

      vel_msg.x_m_s = pos_vel.velocity.north_m_s;
      vel_msg.y_m_s = - pos_vel.velocity.east_m_s;
      vel_msg.z_m_s = - pos_vel.velocity.down_m_s;

      // acc_msg.x_m_s2 = imu.acceleration_frd.forward_m_s2;
      // acc_msg.y_m_s2 = - imu.acceleration_frd.right_m_s2;
      // acc_msg.z_m_s2 = - imu.acceleration_frd.down_m_s2;

      // node->publish_pose(pose_msg);
      node->publish_velocity(vel_msg);
      // node->publish_acceleration(acc_msg);
      rclcpp::spin_some(node);


      // dev printers
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      //     "Vicon pose: [%f, %f, %f | %f, %f, %f]",
      //     vision_msg.position_body.x_m,
      //     vision_msg.position_body.y_m,
      //     vision_msg.position_body.z_m,
      //     vision_msg.angle_body.roll_rad,
      //     vision_msg.angle_body.pitch_rad,
      //     vision_msg.angle_body.yaw_rad);

      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      //     "PX4 pose: [%f, %f, %f | %f, %f, %f]",
      //     pose_msg.x_m,
      //     pose_msg.y_m,
      //     pose_msg.z_m,
      //     pose_msg.roll_deg,
      //     pose_msg.pitch_deg,
      //     pose_msg.yaw_deg);


      // control loop rate
      loop_rate.sleep();

    } // main loop end
  } // connection loop end
}
