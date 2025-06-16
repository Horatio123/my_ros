
#ifndef PROGRESS_CHECKER_HPP_
#define PROGRESS_CHECKER_HPP_

#include <memory>
#include <string>

class ProgressChecker
{
public:
  typedef std::shared_ptr<ProgressChecker> Ptr;

  virtual ~ProgressChecker() {}

  /**
   * @brief Initialize parameters for ProgressChecker
   * @param plugin_name
   */
  virtual void initialize(
      const std::string &plugin_name) = 0;
  /**
   * @brief Checks if the robot has moved compare to previous
   * pose
   * @param current_pose Current pose of the robot
   * @return True if progress is made
   */
  virtual bool check(int num) = 0;
};

#endif // PROGRESS_CHECKER_HPP_
