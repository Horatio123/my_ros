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

BT::NodeStatus CalculateGoal::tick()
{
  Position2D mygoal = {1.1, 2.3};
  setOutput("goal", mygoal);
  return NodeStatus::SUCCESS;
}

BT::NodeStatus PrintTarget::tick()
  {
    auto res = getInput<Position2D>("target");
    if (!res)
    {
      throw RuntimeError("error reading port [target]:", res.error());
    }
    Position2D goal = res.value();
    printf("Target positions: [ %.1f, %.1f ]\n", goal.x, goal.y);
    return NodeStatus::SUCCESS;
  }

}