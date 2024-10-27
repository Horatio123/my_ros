#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;


class CalculateNum : public SyncActionNode
{
public:
  CalculateNum(const std::string& name, const NodeConfiguration& config) :
    SyncActionNode(name, config)
  {}

  NodeStatus tick() override
  {
    int mygoal = 100;
    setOutput("goal", mygoal);
    return NodeStatus::SUCCESS;
  }
  static PortsList providedPorts()
  {
    return {OutputPort<int>("goal")};
  }
};

class PrintNum : public SyncActionNode
{
public:
  PrintNum(const std::string& name, const NodeConfiguration& config) :
    SyncActionNode(name, config)
  {}

  NodeStatus tick() override
  {
    auto res = getInput<int>("target");
    if (!res)
    {
      throw RuntimeError("error reading Num:", res.error());
    }
    int goal = res.value();
    printf("Num: [ %d]\n", goal);
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char* description = "Simply print the target on console...";
    return {InputPort<int>("target", description)};
  }
};


// Action_A has a different constructor than the default one.
class Action_A : public SyncActionNode
{
public:
  // additional arguments passed to the constructor
  Action_A(const std::string& name, const NodeConfiguration& config, int arg1,
           double arg2, std::string arg3) :
    SyncActionNode(name, config), _arg1(arg1), _arg2(arg2), _arg3(arg3)
  {}

  NodeStatus tick() override
  {
    std::cout<< "Action_A: " << _arg1 << " / " << _arg2 << " / " << _arg3 << std::endl;
    return NodeStatus::SUCCESS;
  }
  static PortsList providedPorts()
  {
    return {};
  }

private:
  int _arg1;
  double _arg2;
  std::string _arg3;
};


// Simple tree, used to execute once each action.
static const char* xml_text = R"(

 <root >
     <BehaviorTree>
        <Sequence>
            <CalculateNum   goal="{GoalPosition}" />
            <PrintNum     target="{GoalPosition}" />
            <Action_A/>
            <Action_A_P/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main()
{
  BehaviorTreeFactory factory;

  // A node builder is nothing more than a function pointer to create a
  // std::unique_ptr<TreeNode>.
  // Using lambdas or std::bind, we can easily "inject" additional arguments.
  NodeBuilder builder_A = [](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<Action_A>(name, config, 42, 3.14, "hello world");
  };

  // BehaviorTreeFactory::registerBuilder is the more general way to register a custom node.
  // Not the most user friendly, but definitely the most flexible one.
  factory.registerBuilder<Action_A>("Action_A", builder_A);

  NodeBuilder builder_A_plus = [](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<Action_A>(name, config, 55, 7.77, "hello earth");
  };

  factory.registerNodeType<CalculateNum>("CalculateNum");
  factory.registerNodeType<PrintNum>("PrintNum");


  // BehaviorTreeFactory::registerBuilder is the more general way to register a custom node.
  // Not the most user friendly, but definitely the most flexible one.
  factory.registerBuilder<Action_A>("Action_A_P", builder_A_plus);

  auto tree = factory.createTreeFromText(xml_text);


  // tree.tickRootWhileRunning();
  tree.tickRoot();

  return 0;
}
