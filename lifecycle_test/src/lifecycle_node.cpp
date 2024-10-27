#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class EmptyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  EmptyLifecycleNode()
  : rclcpp_lifecycle::LifecycleNode("lifecycle_node")
  {}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmptyLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}