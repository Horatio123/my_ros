#include "ros_utils/my_lifecycle_node.hpp"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyLifecycleNode>("hz");
  std::cout << "node name is " << node->get_name() << std::endl;
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}
