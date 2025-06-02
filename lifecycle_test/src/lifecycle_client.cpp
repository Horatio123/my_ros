#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/transition_description.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include <chrono>
#include <future>
#include <memory>
#include <string>
using namespace std::chrono_literals;


constexpr char const * node_get_state_topic = "/lifecycle_pb/get_state";
constexpr char const * node_change_state_topic = "/lifecycle_pb/change_state";
const lifecycle_msgs::msg::State unknown_state = lifecycle_msgs::msg::State();

class LifecycleServiceClient : public rclcpp::Node
{
public:
  explicit LifecycleServiceClient(std::string node_name)
  : Node(node_name)
  {
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_change_state_topic);
    node_name_ = node_name;
  }

  // lifecycle_msgs::msg::State
  // get_state_bak(std::chrono::seconds time_out = 10s)
  // {
  //   RCLCPP_INFO(get_logger(), "get_state");

  //   auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  //   if (!client_get_state_->wait_for_service(time_out)) {
  //     return unknown_state;
  //   }

  //   auto future_result = client_get_state_->async_send_request(request);
  //   auto future_status = future_result.wait_for(time_out);

  //   RCLCPP_INFO(get_logger(), "res state");
  //   if (future_status != std::future_status::ready) {
  //     return unknown_state;
  //   }

  //   auto res = future_result.get();
  //   RCLCPP_INFO(get_logger(), "res state2");

  //   RCLCPP_INFO(get_logger(), "state: [%s]", res->current_state.label.c_str());


  //   if (res) {
  //     return res->current_state;
  //   } else {
  //     return unknown_state;
  //   }
  // }

  unsigned int
  get_state(std::chrono::seconds time_out = 100s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
        get_logger(),
        "Service %s is not available.",
        client_get_state_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // 同步调用实现
    auto future_result = client_get_state_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        future_result,
        time_out) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        get_logger(), "Service call failed for node %s", node_name_.c_str());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // 处理同步响应
    auto response = future_result.get();
    if (response) {
      RCLCPP_INFO(
        get_logger(), "Node %s current state: %s",
        node_name_.c_str(), response->current_state.label.c_str());
      return response->current_state.id;
    }
    
    RCLCPP_ERROR(
      get_logger(), "Invalid response from node %s", node_name_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  bool
  change_state(std::uint8_t transition, std::chrono::seconds time_out = 5s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
        get_logger(), 
        "Service %s is unavailable", 
        client_change_state_->get_service_name());
      return false;
    }

    auto future_result = client_change_state_->async_send_request(request);
    
    // 同步等待服务响应
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        future_result,
        time_out) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        get_logger(), 
        "Service call failed for transition %d", 
        static_cast<int>(transition));
      return false;
    }

    // 处理同步响应
    auto response = future_result.get();
    if (response) {
      RCLCPP_INFO(
        get_logger(),
        "Transition %d result: %s",
        static_cast<int>(transition), 
        response->success ? "success" : "failed");
      return response->success;
    }
    
    return false;
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  std::string node_name_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<LifecycleServiceClient>("client");
  client->get_state();
  client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  std::this_thread::sleep_for(10s);
  client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  client->get_state();
  rclcpp::spin_some(client);
  rclcpp::shutdown();
  return 0;
}