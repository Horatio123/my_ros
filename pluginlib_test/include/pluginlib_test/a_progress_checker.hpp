#include <pluginlib_test/progress_checker.hpp>

class AProgressChecker : public ProgressChecker
{
private:
  /* data */
public:
  void initialize(const std::string &plugin_name) override;
  bool check(int num) override;
};
