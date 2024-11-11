#include "dummy_node.hpp"
using namespace std::chrono_literals;

namespace DummyNode{

BT::NodeStatus GripperInterface::close()
{
    std::cout << "GripperInterface::close" << std::endl;
    opened_ = false;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::open()
{
    opened_ = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus SayHello()
{
    std::cout << "Robot says: Hello World" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus CheckBattery() {
    std::cout << "[ Battery: OK ]" << std::endl;
    rclcpp::Rate loop_rate(2s);
    loop_rate.sleep();

    // return BT::NodeStatus::RUNNING;
    return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus CheckTemperature()
{
    std::cout << "[ Temperature: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

}