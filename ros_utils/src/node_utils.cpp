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

#include "ros_utils/node_utils.hpp"
#include <chrono>
#include <string>
#include <algorithm>
#include <cctype>

using std::chrono::high_resolution_clock;
using std::to_string;
using std::string;
using std::replace_if;
using std::isalnum;

namespace util
{

string sanitize_node_name(const string & potential_node_name)
{
  string node_name(potential_node_name);
  // read this as `replace` characters in `node_name` `if` not alphanumeric.
  // replace with '_'
  replace_if(
    begin(node_name), end(node_name),
    [](auto c) {return !isalnum(c);},
    '_');
  return node_name;
}

string add_namespaces(const string & top_ns, const string & sub_ns)
{
  if (!top_ns.empty() && top_ns.back() == '/') {
    if (top_ns.front() == '/') {
      return top_ns + sub_ns;
    } else {
      return "/" + top_ns + sub_ns;
    }
  }

  return top_ns + "/" + sub_ns;
}

std::string time_to_string(size_t len)
{
  string output(len, '0');  // prefill the string with zeros
  auto timepoint = high_resolution_clock::now();
  auto timecount = timepoint.time_since_epoch().count();
  auto timestring = to_string(timecount);
  std::cout <<"----timestring is "<< timestring << std::endl;

  if (timestring.length() >= len) {
    // if `timestring` is shorter, put it at the end of `output`
    output.replace(
      0, len,
      timestring,
      timestring.length() - len, len);
  } else {
    // if `output` is shorter, just copy in the end of `timestring`
    output.replace(
      len - timestring.length(), timestring.length(),
      timestring,
      0, timestring.length());
  }
  return output;
}

std::string generate_internal_node_name(const std::string & prefix)
{
  return sanitize_node_name(prefix) + "_" + time_to_string(8);
}

rclcpp::Node::SharedPtr generate_internal_node(const std::string & prefix)
{
  auto options =
    rclcpp::NodeOptions()
    .start_parameter_services(false)
    .start_parameter_event_publisher(false)
    .arguments({"--ros-args", "-r", "__node:=" + generate_internal_node_name(prefix), "--"});
  return rclcpp::Node::make_shared("_", options);
}

rclcpp::NodeOptions
get_node_options_default(bool allow_undeclared, bool declare_initial_params)
{
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(allow_undeclared);
  options.automatically_declare_parameters_from_overrides(declare_initial_params);
  return options;
}

}  // namespace nav2_util
