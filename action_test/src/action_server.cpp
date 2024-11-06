#include "rclcpp/rclcpp.hpp"
#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>
using namespace std::placeholders;
using Fibonacci=action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class ActionServer: public rclcpp::Node
{
public:
    ActionServer():Node("action_server") {
        RCLCPP_INFO(this->get_logger(), "create action_server");
        this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, 
        "fibonacci_server", 
        std::bind(&ActionServer::handle_goal,this, _1, _2),
        std::bind(&ActionServer::handle_cancel,this, _1),
        std::bind(&ActionServer::handle_accept, this, _1));
    };

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
    
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Fibonacci::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
//     rclcpp_action::GoalResponse handle_goal(
//     const rclcpp_action::GoalUUID & uuid,
//     std::shared_ptr<const Fibonacci::Goal> goal)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
//     (void)uuid;
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }


    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    void handle_accept(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle){
        std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
    };

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto server = std::make_shared<ActionServer>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}