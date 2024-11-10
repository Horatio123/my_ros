#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace DummyNode{
using BT::NodeStatus;
NodeStatus CheckBattery();
NodeStatus CheckTemperature();
NodeStatus SayHello();

class GripperInterface
{
private:
    bool opened_;
public:
    GripperInterface():opened_(true){}
    NodeStatus open();
    NodeStatus close();
};

}