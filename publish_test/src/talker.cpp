#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
private:
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string animal_;
  void timer_callback()
  {
    this->get_parameter("animal", animal_);
    auto message = std_msgs::msg::String();
    message.data = "Hello " + animal_ + " " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "publish '%s'", message.data.c_str());
    RCLCPP_DEBUG(this->get_logger(), "this is test for DEBUG");
    publisher_->publish(message);
  }

public:
  Talker() : Node("talker"), count_{0}
  {
    this->declare_parameter("animal", "cat");
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));
  }
};

int main(int argc, char **argv)
{
  printf("hello world publish_test package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}

// ros2 run publish_test talker --ros-args -p animal:=lion
// ros2 run publish_test talker --ros-args -p animal:=lion --log-level DEBUG