#include <cstdio>
#include <iostream>
#include <pluginlib_test/a_progress_checker.hpp>
#include "pluginlib/class_list_macros.hpp"

void AProgressChecker::initialize(const std::string &plugin_name)
{
  std::cout << "AProgressChecker::initialize" << std::endl;
  std::cout << "plugin_name: " << plugin_name << std::endl;
}

bool AProgressChecker::check(int num)
{
  std::cout << "AProgressChecker::check" << std::endl;
  if (num > 10)
  {
    std::cout << "num is " << num << " num > 10" << std::endl;
    return true;
  }
  else
  {
    std::cout << "num is " << num << " num <= 10" << std::endl;
    return false;
  }
}

PLUGINLIB_EXPORT_CLASS(AProgressChecker, ProgressChecker)
