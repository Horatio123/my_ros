// Copyright (c) 2019 Intel Corporation
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

#include <memory>

#include "ros_utils/node_thread.hpp"

namespace util
{

NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(node_base)
{
  thread_ = std::make_unique<std::thread>(
    [&]()
    {
      executor_.add_node(node_);
      executor_.spin();
      executor_.remove_node(node_);
    });
}


NodeThread::~NodeThread()
{
  executor_.cancel();
  thread_->join();
}

}  // namespace util
