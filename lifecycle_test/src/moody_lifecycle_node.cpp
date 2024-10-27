#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class MoodyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MoodyLifecycleNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
  {}

  size_t number_of_callbacks = 0;

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    RCLCPP_INFO(this->get_logger(), "state id: '%d'", get_current_state().id());

    RCLCPP_INFO(this->get_logger(), "state: '%s'", get_current_state().label().c_str());
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    return CallbackReturn::SUCCESS;
  }

};



int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world lifecycle_test package moody_lifecycle_node node\n");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoodyLifecycleNode>("moody_lifecycle_node");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
