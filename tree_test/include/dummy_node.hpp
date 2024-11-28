#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
using namespace BT;
struct Position2D
{
  double x, y;
};

namespace BT
{
  template <>
  inline Position2D convertFromString(StringView str)
  {
    printf("Converting string: \"%s\"\n", str.data());

    // real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
      throw RuntimeError("invalid input)");
    }
    else
    {
      Position2D output;
      output.x = convertFromString<double>(parts[0]);
      output.y = convertFromString<double>(parts[1]);
      return output;
    }
  }
} // end namespace BT

namespace DummyNode
{
  NodeStatus CheckBattery();
  NodeStatus CheckTemperature();
  NodeStatus SayHello();

  class GripperInterface
  {
  private:
    bool opened_;

  public:
    GripperInterface() : opened_(true) {}
    NodeStatus open();
    NodeStatus close();
  };
  class CalculateGoal : public SyncActionNode
  {
  public:
    CalculateGoal(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
      return {OutputPort<Position2D>("goal")};
    }
  };

  class PrintTarget : public SyncActionNode
  {
  public:
    PrintTarget(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
      // Optionally, a port can have a human readable description
      const char *description = "Simply print the target on console...";
      return {InputPort<Position2D>("target", description)};
    }
  };

}