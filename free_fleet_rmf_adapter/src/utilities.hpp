/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef FF_RMF_ADAPTER__SRC__UTILITIES_HPP
#define FF_RMF_ADAPTER__SRC__UTILITIES_HPP

#include <string>
#include <memory>

#include <rclcpp/node.hpp>

namespace free_fleet {

std::string generate_random_task_id(size_t length);

//==============================================================================

template<typename T>
void get_parameter_or_default(
    std::shared_ptr<rclcpp::Node> node, 
    const std::string& param_name, 
    T& value)
{
  const auto& node = adapter->node();
  value = node->declare_parameter(param_name, value);
}

//==============================================================================

bool get_string_parameter_mandatory(
    std::shared_ptr<rclcpp::Node> node, 
    const std::string& param_name, 
    std::string& value)
{
  value = node->declare_parameter(param_name, std::string());
  if (value.empty())
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Missing [%s] parameter", param_name.c_str());
    return false;
  }
  return true;
}

//==============================================================================

} // namespace free_fleet

#endif // FF_RMF_ADAPTER__SRC__UTILITIES_HPP
