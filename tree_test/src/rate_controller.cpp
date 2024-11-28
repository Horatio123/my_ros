// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <string>

#include "rate_controller.hpp"

namespace nav2_behavior_tree
{

  RateController::RateController(
      const std::string &name,
      const BT::NodeConfiguration &conf)
      : BT::DecoratorNode(name, conf),
        first_time_(false)
  {
    double hz = 0.1;
    getInput("hz", hz);
    std::cout << "--------hz is:  " << hz << std::endl;
    period_ = 1.0 / hz;
  }

  BT::NodeStatus RateController::tick()
  {
    // std::cout << "--------status is: " << status() << std::endl;
    if (status() == BT::NodeStatus::IDLE)
    {
      // Reset the starting point since we're starting a new iteration of
      // the rate controller (moving from IDLE to RUNNING)
      start_ = std::chrono::high_resolution_clock::now();
      first_time_ = true;
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Determine how long its been since we've started this iteration
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - start_;

    // Now, get that in seconds
    typedef std::chrono::duration<float> float_seconds;
    auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

    // The child gets ticked the first time through and any time the period has
    // expired. In addition, once the child begins to run, it is ticked each time
    // 'til completion

    //|| (child_node_->status() == BT::NodeStatus::RUNNING)
    if (first_time_ || seconds.count() >= period_)
    {
      std::cout << "--------seconds.count is:  " << seconds.count() << std::endl;
      std::cout << "--------period_ is:  " << period_ << std::endl;
      std::cout << "--------before child tick " << first_time_ << std::endl;
      first_time_ = false;

      const BT::NodeStatus child_state = child_node_->executeTick();

      std::cout << "--------child_state is: " << child_state << std::endl;
      start_ = std::chrono::high_resolution_clock::now(); // Reset the timer

      switch (child_state)
      {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:

        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
      }
    }

    return status();
  }

} // namespace nav2_behavior_tree

// #include "behaviortree_cpp_v3/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<nav2_behavior_tree::RateController>("RateController");
// }
