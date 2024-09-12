#include "rclcpp/rclcpp.hpp"
#include "gripper_control/gripper_control.hpp"




/**
 * Print error message when no port is provided as argument. TODO
*/
void usage(const std::string &bin_name) {
  std::cerr
      << "ERROR: No serial port provided.\n"
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      // << " For TCP : tcp://[server_host][:server_port]\n"
      // << " For UDP : udp://[bind_host][:bind_port]\n"
      << " Serial : /path/to/serial/dev[:baudrate]\n"
      << "For example: /dev/ttyUSB0\n";
      // << "For example, to connect to the simulator use URL: udp://:14540\n";
}


// command line argument: serial port, i.e. /dev/ttyUSB0
int main(int argc, char *argv[])
{
  // check command line input
  if (argc < 2)
  {
    usage(argv[0]);
    return 1;
  }

  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto gripper_control_node = std::make_shared<Gripper>(argv[1]);

  // spin node if initialization was successful
  if(rclcpp::ok()) {
    RCLCPP_INFO(gripper_control_node->get_logger(), "Gripper control node ready.");
    rclcpp::spin(gripper_control_node);
  }

  // serial.closeDevice(); // TODO necessary? add this to node (destructor)?
  rclcpp::shutdown();
  return 0;
}