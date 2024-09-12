#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


class DevPublisher : public rclcpp::Node
{
public:
  DevPublisher() : Node("dev_pub_node")
  {
    // create publisher
    pub_ = this->create_publisher<std_msgs::msg::Bool>("dev_publisher", 10);

    // create timer
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DevPublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    // create message
    auto msg = std_msgs::msg::Bool();

    // fill message
    msg.data = true;

    // publish message
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};




int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize the node
  auto dev_pub_node = std::make_shared<DevPublisher>();

  // info msg that node is running
  RCLCPP_INFO(dev_pub_node->get_logger(), "Alive and kicking.");
  // run the node until ros is not ok
  rclcpp::spin(dev_pub_node);

  rclcpp::shutdown();
  return 0;
}