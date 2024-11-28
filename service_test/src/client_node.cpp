#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
using namespace std::chrono_literals;

class Client : public rclcpp::Node
{
private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

public:
  Client() : Node("client")
  {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  };
  // auto get_client(){
  //     return client_;
  // }
  auto send_request(int a, int b)
  {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;
    while (!client_->wait_for_service(1s))
    {
      RCLCPP_INFO(this->get_logger(), "service is not available");
    }
    auto result_future = client_->async_send_request(request);
    return result_future;
    // result_future.wait();
    // if (result_future.valid())
    // {
    //     auto result = result_future.get();
    //     RCLCPP_INFO(this->get_logger(), "result = %ld", result->sum);
    // }
  }
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  // auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  // request->a = 1;
  // request->b = 7;
  if (argc != 3)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: ros2 run service_test client_node X Y");
    return 1;
  }

  auto client = std::make_shared<Client>();
  // auto result = client->get_client()->async_send_request(request);
  int a = atoll(argv[1]);
  int b = atoll(argv[2]);
  auto result = client->send_request(a, b);
  if (rclcpp::spin_until_future_complete(client, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(client->get_logger(), "Sum: %ld", result.get()->sum);
  }
  else
  {
    RCLCPP_ERROR(client->get_logger(), "Failed to call service add_two_ints");
  }
  rclcpp::shutdown();
  return 0;
}

// root@docker-desktop:/home/ros/my_ros# ros2 run service_test client_node 4 88
// 2024-11-03 10:09:25.821 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7414: open_and_lock_file failed -> Function open_port_internal
// [INFO] [1730628566.852943400] [client]: service is not available
// [INFO] [1730628567.297621800] [client]: Sum: 92