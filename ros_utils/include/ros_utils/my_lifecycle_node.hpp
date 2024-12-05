#include "ros_utils/lifecycle_node.hpp"

class MyLifecycleNode : public util::LifecycleNode
{
private:
  std::string my_home_;
public:
  MyLifecycleNode(std::string my_home);
  ~MyLifecycleNode();
};

MyLifecycleNode::MyLifecycleNode(std::string my_home) : util::LifecycleNode("MyNode", "", true), my_home_{my_home}
{
  std::cout << "start MyNode, my home is " << my_home_ << std::endl;
}

MyLifecycleNode::~MyLifecycleNode()
{
  std::cout << "leave my home " << my_home_ << std::endl;
}
