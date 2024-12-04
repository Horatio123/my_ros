#include <cstdio>
#include "utils/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::string node_name = node_utils::sanitize_node_name("123[]qwe!@#$^&*<>/foo/bar");
  std::cout <<"node name is "<< node_name << std::endl;

  std::string time_str = node_utils::time_to_string(8);
  std::cout <<"time is "<< time_str << std::endl;

  rclcpp::init(argc, argv);
  auto node = node_utils::generate_internal_node("test_node");
  printf("start node\n");
  std::cout <<"node name is "<< node->get_name() << std::endl;
  rclcpp::spin_some(node);
  rclcpp::shutdown();

  printf("hello world utils package\n");
  return 0;
}
