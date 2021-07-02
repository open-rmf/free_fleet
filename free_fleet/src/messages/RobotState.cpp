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
#include <free_fleet/messages/RobotState.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class RobotState::Implementation
{
public:

  rmf_traffic::Time time;

  std::string name;

  std::string model;

  std::optional<TaskId> task_id;

  RobotMode mode;

  double battery_percent;

  Location location;

  std::size_t target_path_index;
};

//==============================================================================
RobotState::RobotState(
  rmf_traffic::Time time,
  const std::string& name,
  const std::string& model,
  std::optional<TaskId> task_id,
  const RobotMode& mode,
  double battery_percent,
  const Location& location,
  std::size_t target_path_index)
{
  std::string error_message;

  if (name.empty())
    error_message = "Name in RobotState cannot be empty.";
  else if (model.empty())
    error_message = "Model in RobotState cannot be empty.";
  else if (battery_percent < 0.0 || battery_percent > 1.0)
    error_message = "Battery percentage in RobotState must be represented "
      "between 0.0 (empty) and 1.0 (full).";

  if (!error_message.empty())
  {
    fferr << error_message << "\n";
    throw std::invalid_argument(error_message);
  }

  _pimpl = rmf_utils::make_impl<Implementation>(Implementation{
    time,
    name,
    model,
    std::move(task_id),
    mode,
    battery_percent,
    location,
    target_path_index});
}

//==============================================================================
rmf_traffic::Time RobotState::time() const
{
  return _pimpl->time;
}

//==============================================================================
const std::string& RobotState::name() const
{
  return _pimpl->name;
}

//==============================================================================
const std::string& RobotState::model() const
{
  return _pimpl->model;
}

//==============================================================================
std::optional<TaskId> RobotState::task_id() const
{
  return _pimpl->task_id;
}

//==============================================================================
const RobotMode& RobotState::mode() const
{
  return _pimpl->mode;
}

//==============================================================================
double RobotState::battery_percent() const
{
  return _pimpl->battery_percent;
}

//==============================================================================
const Location& RobotState::location() const
{
  return _pimpl->location;
}

//==============================================================================
std::size_t RobotState::target_path_index() const
{
  return _pimpl->target_path_index;
}

//==============================================================================
bool operator==(const RobotState& lhs, const RobotState& rhs)
{
  if (lhs.time() == rhs.time() &&
    lhs.name() == rhs.name() &&
    lhs.model() == rhs.model() &&
    lhs.task_id() == rhs.task_id() &&
    lhs.mode() == rhs.mode() &&
    abs(lhs.battery_percent() - rhs.battery_percent()) < 1e-3 &&
    lhs.location() == rhs.location() &&
    lhs.target_path_index() == rhs.target_path_index())
    return true;
  return false;
}

//==============================================================================
bool operator!=(const RobotState& lhs, const RobotState& rhs)
{
  return !(lhs == rhs);
}

//==============================================================================
} // namespace messages
} // namespace free_fleet
