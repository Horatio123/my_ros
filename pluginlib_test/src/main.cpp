#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib_test/progress_checker.hpp"


int main(int /*argc*/, char ** /*argv*/)
{
  pluginlib::ClassLoader<ProgressChecker> progress_checker_loader_("pluginlib_test", "ProgressChecker");
  ProgressChecker::Ptr progress_checker_ = progress_checker_loader_.createUniqueInstance("AProgressChecker");
  progress_checker_->initialize("first_ros_plugin");
  progress_checker_->check(11);
}