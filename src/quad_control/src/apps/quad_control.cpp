#include "rclcpp/rclcpp.hpp"

#include "quad_control/quad.hpp"




/**
 * Print error message when no port is provided as argument.
*/
void usage(const std::string &bin_name) {
  std::cerr
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}


int main(int argc, char *argv[])
{
  // check command line input
  if (argc < 2)
  {
    std::cerr << "Port not provided.\n";
    usage(argv[0]);
    return 1;
  }

  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto quad_control_node = std::make_shared<Quad>(argv[1]);

  // spin node if initialization was successful
  if(rclcpp::ok()) {
    RCLCPP_INFO(quad_control_node->get_logger(), "Ready.");
    rclcpp::spin(quad_control_node);
  }

  rclcpp::shutdown();
  return 0;
}