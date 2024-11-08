#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalhandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class ActionClient: public rclcpp::Node
{
private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    void send_goal(){
        this->timer_->cancel();
        while (!client_->wait_for_action_server(1s))
        {
            RCLCPP_INFO(this->get_logger(), "wait for fibonacci action");
        }
        auto goal = Fibonacci::Goal();
        goal.order = 10;

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&ActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&ActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&ActionClient::result_callback, this, _1);

        client_->async_send_goal(goal, send_goal_options);

    };

    void goal_response_callback(std::shared_future<std::shared_ptr<GoalhandleFibonacci>> future){
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "goal was reject");
        } else {
            RCLCPP_INFO(this->get_logger(), "goal was accept");
        };
    };
    void feedback_callback(std::shared_ptr<GoalhandleFibonacci>, 
    std::shared_ptr<const example_interfaces::action::Fibonacci_Feedback> feedback){
        RCLCPP_INFO(this->get_logger(), "caculate num is %d", feedback->sequence.back());
    };
    void result_callback(const GoalhandleFibonacci::WrappedResult &result){
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "get result success");
            for (auto num : result.result->sequence) {
                RCLCPP_INFO(this->get_logger(), "get result %d", num);
            };
            
        }
    };

public:
    ActionClient():Node("action_client"){
        client_ = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci"
        );
        timer_ = this->create_wall_timer(1s, std::bind(&ActionClient::send_goal, this));
    };
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionClient>());
    rclcpp::shutdown();
    return 0;
}
