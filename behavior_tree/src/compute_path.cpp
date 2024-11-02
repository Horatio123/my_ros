#include <cstdio>
#include "behavior_tree/action_server.hpp"
#include "behavior_msg/action/compute_path.hpp"

class ComputePathActionServer : public ActionServer<behavior_msg::action::ComputePath>
{
public:
  ComputePathActionServer()
  : ActionServer("compute_path")
  {
    RCLCPP_INFO(this->get_logger(), "create server");
    RCLCPP_DEBUG(this->get_logger(), "create server test for debug");
  }

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<behavior_msg::action::ComputePath>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<behavior_msg::action::ComputePath::Result>();
    result->path.poses.resize(1);
    result->path.poses[0].pose.position.x = goal->pose.pose.position.x;
    goal_handle->succeed(result);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComputePathActionServer>());
  rclcpp::shutdown();
  return 0;
}