#include "ros_utils/service_client.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "ros_utils/lifecycle_service_client.hpp"
#include "ros_utils/lifecycle_utils.hpp"
using namespace std::chrono_literals;
using lifecycle_msgs::msg::Transition;

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  std::string my_node_name = "MyNode";

  auto get_state_client = util::ServiceClient<lifecycle_msgs::srv::GetState>(my_node_name + "/get_state");
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  const std::chrono::seconds service_call_timeout = 10s;
  auto result = get_state_client.invoke(request, service_call_timeout);
  auto label = result->current_state.label;
  std::cout << "ServiceClient get state is " << label << std::endl;

  auto lifecycle_client = util::LifecycleServiceClient(my_node_name);
  lifecycle_client.change_state(Transition::TRANSITION_CONFIGURE, service_call_timeout);
  lifecycle_client.change_state(Transition::TRANSITION_ACTIVATE, service_call_timeout);

  auto id = lifecycle_client.get_state(service_call_timeout);
  std::cout << "LifecycleServiceClient get state is " << std::to_string(id) << std::endl;

  util::reset_lifecycle_nodes(my_node_name, service_call_timeout);
  
  auto id2 = lifecycle_client.get_state(service_call_timeout);
  std::cout << "LifecycleServiceClient get state is " << std::to_string(id2) << std::endl;


  rclcpp::shutdown();
}
