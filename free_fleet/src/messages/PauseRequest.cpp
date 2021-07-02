/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <free_fleet/Console.hpp>
#include <free_fleet/messages/PauseRequest.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class PauseRequest::Implementation
{
public:

  std::string robot_name;

  TaskId task_id;
};

//==============================================================================
PauseRequest::PauseRequest(const std::string& robot_name, TaskId task_id)
{
  std::string error_message;

  if (robot_name.empty())
    error_message = "robot_name in PauseRequest must not be empty.";

  if (!error_message.empty())
  {
    fferr << error_message << "\n";
    throw std::invalid_argument(error_message);
  }

  _pimpl = rmf_utils::make_impl<Implementation>(Implementation{
    robot_name,
    task_id});
}

//==============================================================================
const std::string& PauseRequest::robot_name() const
{
  return _pimpl->robot_name;
}

//==============================================================================
TaskId PauseRequest::task_id() const
{
  return _pimpl->task_id;
}

//==============================================================================
bool operator==(const PauseRequest& lhs, const PauseRequest& rhs)
{
  if (lhs.robot_name() == rhs.robot_name() &&
    lhs.task_id() == rhs.task_id())
    return true;
  return false;
}

//==============================================================================
bool operator!=(const PauseRequest& lhs, const PauseRequest& rhs)
{
  return !(lhs == rhs);
}

//==============================================================================
} // namespace messages
} // namespace free_fleet
