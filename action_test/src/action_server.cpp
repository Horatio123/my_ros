#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

class ActionServer:public rclcpp::Node
{
private:
    rclcpp_action::Server<Fibonacci>::SharedPtr server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID, std::shared_ptr< const Fibonacci::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "reveice goal %d", goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>){
        RCLCPP_INFO(this->get_logger(), "cancel operation");
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
        std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
    };
    void execute(std::shared_ptr<GoalHandle> goal_handle) {
        rclcpp::Rate loop_rate(1);
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();
        auto & sequence = feedback->sequence;

        sequence.push_back(0);
        sequence.push_back(1);

        for (int i = 1; (i < goal->order) && rclcpp::ok(); i++)
        {
            sequence.push_back(sequence.at(i - 1) + sequence.at(i));
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "feedback %d", sequence.back());
            loop_rate.sleep();
        }
        if (rclcpp::ok())
        {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "succeeded %d", sequence.back());
        }
    }
    
public:
    ActionServer():Node("fibonacci_server"){
        RCLCPP_INFO(this->get_logger(), "create action_server");
        server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&ActionServer::handle_goal, this, _1, _2),
            std::bind(&ActionServer::handle_cancel, this, _1),
            std::bind(&ActionServer::handle_accepted, this, _1)
        );
    };
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionServer>());
    rclcpp::shutdown();
    return 0;
}
