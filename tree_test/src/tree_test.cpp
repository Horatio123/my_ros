#include "behaviortree_cpp_v3/bt_factory.h"

#define MANUAL_STATIC_LINKING

#ifdef MANUAL_STATIC_LINKING
#include "dummy_node.hpp"
#include "rate_controller.hpp"
#include "pipeline_sequence.hpp"
#endif

using namespace BT;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 *
 * In this tutorial, we will learn how to create custom ActionNodes.
 * It is important to remember that NodeTree are just a way to
 * invoke callbacks (called tick() ). These callbacks are implemented by the user.
 */

// clang-format off
static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <PipelineSequence name="root_sequence">
            <RateController hz="0.1">
              <CheckBattery   name="battery_ok"/>
            </RateController>
            <OpenGripper    name="open_gripper"/>
            <CloseGripper   name="close_gripper"/>
            <CalculateGoal   goal="{GoalPosition}" />
            <PrintTarget     target="{GoalPosition}" />
            <SetBlackboard   output_key="OtherGoal" value="-1;3" />
            <PrintTarget     target="{OtherGoal}" />
        </PipelineSequence>
     </BehaviorTree>

 </root>
 )";

// clang-format on

int main()
{
  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  /* There are two ways to register nodes:
   *    - statically, i.e. registering all the nodes one by one.
   *    - dynamically, loading the TreeNodes from a shared library (plugin).
   * */

#ifdef MANUAL_STATIC_LINKING
  // Note: the name used to register should be the same used in the XML.
  // Note that the same operations could be done using DummyNodes::RegisterNodes(factory)

  using namespace DummyNode;

  // The recommended way to create a Node is through inheritance.
  // Even if it requires more boilerplate, it allows you to use more functionalities
  // like ports (we will discuss this in future tutorials).
  // factory.registerNodeType<ApproachObject>("ApproachObject");

  // Registering a SimpleActionNode using a function pointer.
  // you may also use C++11 lambdas instead of std::bind
  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

  // You can also create SimpleActionNodes using methods of a class
  GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper",
                               std::bind(&GripperInterface::open, &gripper));
  factory.registerSimpleAction("CloseGripper",
                               std::bind(&GripperInterface::close, &gripper));

  factory.registerNodeType<nav2_behavior_tree::RateController>("RateController");
  factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");
  BT::Blackboard::Ptr blackboard_ = BT::Blackboard::create();
  blackboard_->set<int>("number_of_cat", 2);
#else
  // Load dynamically a plugin and register the TreeNodes it contains
  // it automated the registering step.
  factory.registerFromPlugin("./libdummy_nodes_dyn.so");
#endif

  // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
  // The currently supported format is XML.
  // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromText(xml_text, blackboard_);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  std::cout << "+++++++++++first tick" << std::endl;
  tree.tickRootWhileRunning();
  // std::cout << "++++++++++++second tick" << std::endl;
  // tree.tickRootWhileRunning();
  int num;
  blackboard_->get("number_of_cat", num);
  std::cout << "main number_of_cat is " << num << std::endl;

  return 0;
}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/
