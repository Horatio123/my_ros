#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class Service : public rclcpp::Node
{
private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr serivce_;
  void add(example_interfaces::srv::AddTwoInts::Request::SharedPtr request, example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
  {
    int a = request->a;
    int b = request->b;
    RCLCPP_INFO(this->get_logger(), "get call %ld, %ld", a, b);
    response->sum = a + b;
    RCLCPP_INFO(this->get_logger(), "sum is  %ld", response->sum);
  }

public:
  Service() : Node("serivce")
  {
    serivce_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&Service::add, this, std::placeholders::_1, std::placeholders::_2));
  };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start service");
  rclcpp::spin(std::make_shared<Service>());
  rclcpp::shutdown();
}
