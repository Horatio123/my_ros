#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Listener : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listener_;
  void listen_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "get '%s'", msg->data.c_str());
  }

public:
  Listener() : Node("listener")
  {
    RCLCPP_INFO(this->get_logger(), "start listener");
    listener_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&Listener::listen_callback, this, _1));
  };
  ~Listener()
  {
    RCLCPP_INFO(this->get_logger(), "stop listener");
  };
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
