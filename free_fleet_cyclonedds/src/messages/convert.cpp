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

#include <sstream>
#include <free_fleet/Types.hpp>
#include <free_fleet/Console.hpp>

#include "convert.hpp"

namespace free_fleet {
namespace cyclonedds {

//==============================================================================
inline std::nullopt_t convert_error(const std::string& error_message)
{
  fferr << error_message << "\n";
  return std::nullopt;
}

//==============================================================================
char* dds_string_alloc_and_copy(const std::string& _str)
{
  char* ptr = dds_string_alloc(_str.length());
  for (size_t i = 0; i < _str.length(); ++i)
  {
    ptr[i] = _str[i];
  }
  return ptr;
}

//==============================================================================
std::optional<MiddlewareMessages_Time> convert(const rmf_traffic::Time& input)
{
  auto time = input.time_since_epoch();
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(time);
  auto nanosec =
    std::chrono::duration_cast<std::chrono::nanoseconds>(time - sec);

  MiddlewareMessages_Time output;
  output.sec = static_cast<int32_t>(sec.count());
  output.nanosec = static_cast<uint32_t>(nanosec.count());
  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_Location> convert(
  const messages::Location& input)
{
  MiddlewareMessages_Location output;
  output.map_name = dds_string_alloc_and_copy(input.map_name());

  if (!output.map_name)
    return convert_error("Location::map_name must not be null.");

  output.x = input.coordinates()[0];
  output.y = input.coordinates()[1];
  output.yaw_available = false;

  if (bool yaw_available = input.yaw().has_value())
  {
    output.yaw_available = yaw_available;
    output.yaw = input.yaw().value();
  }
  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_Waypoint> convert(
  const messages::Waypoint& input)
{
  MiddlewareMessages_Waypoint output;
  output.index = static_cast<uint32_t>(input.index());

  auto dds_loc = convert(input.location());
  if (!dds_loc.has_value())
    return convert_error("failed to convert Waypoint::Location.");

  output.location = dds_loc.value();
  output.wait_until_available = false;

  if (input.wait_until().has_value())
  {
    output.wait_until_available = true;

    auto wait_until = convert(input.wait_until().value());
    if (!wait_until.has_value())
      return convert_error("failed to convert Waypont::wait_until.");

    output.wait_until = wait_until.value();
  }
  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_RobotMode> convert(
  const messages::RobotMode& input)
{
  using Mode = messages::RobotMode::Mode;

  MiddlewareMessages_RobotMode output;
  switch (input.mode())
  {
    case Mode::Idle:
      output.mode = MiddlewareMessages_RobotMode_Constants_Idle;
      break;
    case Mode::Charging:
      output.mode = MiddlewareMessages_RobotMode_Constants_Charging;
      break;
    case Mode::Moving:
      output.mode = MiddlewareMessages_RobotMode_Constants_Moving;
      break;
    case Mode::Paused:
      output.mode = MiddlewareMessages_RobotMode_Constants_Paused;
      break;
    case Mode::Waiting:
      output.mode = MiddlewareMessages_RobotMode_Constants_Waiting;
      break;
    case Mode::Emergency:
      output.mode = MiddlewareMessages_RobotMode_Constants_Emergency;
      break;
    case Mode::Docking:
      output.mode = MiddlewareMessages_RobotMode_Constants_Docking;
      break;
    case Mode::Error:
      output.mode = MiddlewareMessages_RobotMode_Constants_Error;
      break;
    case Mode::Undefined:
      output.mode = MiddlewareMessages_RobotMode_Constants_Undefined;
      break;
    case Mode::Custom:
      output.mode = MiddlewareMessages_RobotMode_Constants_Custom;
      break;
    default:
      output.mode = MiddlewareMessages_RobotMode_Constants_Undefined;
  }

  output.info = dds_string_alloc_and_copy(input.info());
  if (!output.info)
    return convert_error("RobotMode::info must not be null.");

  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_PauseRequest> convert(
  const messages::PauseRequest& input)
{
  MiddlewareMessages_PauseRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());

  if (!output.robot_name)
    return convert_error("PauseRequest::robot_name must not be null.");

  output.task_id = static_cast<uint32_t>(input.task_id());
  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_ResumeRequest> convert(
  const messages::ResumeRequest& input)
{
  MiddlewareMessages_ResumeRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());

  if (!output.robot_name)
    return convert_error("ResumeRequest::robot_name must not be null.");

  output.task_id = static_cast<uint32_t>(input.task_id());
  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_DockRequest> convert(
  const messages::DockRequest& input)
{
  MiddlewareMessages_DockRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());

  if (!output.robot_name)
    return convert_error("DockRequest::robot_name must not be null.");

  output.task_id = static_cast<uint32_t>(input.task_id());
  output.dock_name = dds_string_alloc_and_copy(input.dock_name());

  if (!output.dock_name)
    return convert_error("DockRequest::dock_name must not be null.");

  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_NavigationRequest> convert(
  const messages::NavigationRequest& input)
{
  MiddlewareMessages_NavigationRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());

  if (!output.robot_name)
    return convert_error("NavigationRequest::robot_name must not be null.");

  output.task_id = static_cast<uint32_t>(input.task_id());

  const std::size_t path_elem_num = input.path().size();
  output.path._maximum = static_cast<uint32_t>(path_elem_num);
  output.path._length = static_cast<uint32_t>(path_elem_num);
  output.path._buffer =
    dds_sequence_MiddlewareMessages_Waypoint_allocbuf(path_elem_num);

  if (!output.path._buffer)
    return convert_error("NavigationRequest::path::_buffer must not be null.");

  for (std::size_t i = 0; i < path_elem_num; ++i)
  {
    auto wp = convert(input.path()[i]);
    if (!wp.has_value())
    {
      std::stringstream ss;
      ss << "failed to convert Waypoint " << i
         << " in NavigationRequest::path.\n";
      return convert_error(ss.str());
    }

    output.path._buffer[i] = wp.value();
  }
  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_RelocalizationRequest> convert(
  const messages::RelocalizationRequest& input)
{
  MiddlewareMessages_RelocalizationRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());
  if (!output.robot_name)
    return convert_error("RelocalizationRequest::robot_name must not be null.");

  output.task_id = static_cast<uint32_t>(input.task_id());

  auto loc = convert(input.location());
  if (!loc.has_value())
    return convert_error("failed to convert RelocalizationRequest::location.");
  output.location = loc.value();

  output.last_visited_waypoint_index =
    static_cast<uint32_t>(input.last_visited_waypoint_index());
  return output;
}

//==============================================================================
std::optional<MiddlewareMessages_RobotState> convert(
  const messages::RobotState& input)
{
  MiddlewareMessages_RobotState output;

  auto time = convert(input.time());
  if (!time.has_value())
    return convert_error("failed to convert RobotState::time.");
  output.time = time.value();

  output.name = dds_string_alloc_and_copy(input.name());
  if (!output.name)
    return convert_error("RobotState::name must not be null.");

  output.model = dds_string_alloc_and_copy(input.model());
  if (!output.model)
    return convert_error("RobotState::model must not be null.");

  output.task_id_available = false;
  if (input.task_id().has_value())
  {
    output.task_id_available = true;
    output.task_id = static_cast<uint32_t>(input.task_id().value());
  }

  auto mode = convert(input.mode());
  if (!mode.has_value())
    return convert_error("failed to convert RobotState::mode.");
  output.mode = mode.value();

  output.battery_percent = input.battery_percent();

  auto loc = convert(input.location());
  if (!loc.has_value())
    return convert_error("failed to convert RobotState::location.");
  output.location = loc.value();

  output.target_path_index_available = false;
  if (input.target_path_index().has_value())
  {
    output.target_path_index_available = true;
    output.target_path_index =
      static_cast<uint32_t>(input.target_path_index().value());
  }

  return output;
}

//==============================================================================
std::optional<rmf_traffic::Time> convert(const MiddlewareMessages_Time& input)
{
  auto output = rmf_traffic::Time() +
    std::chrono::seconds(input.sec) +
    std::chrono::nanoseconds(input.nanosec);
  return output;
}

//==============================================================================
std::optional<messages::Location> convert(
  const MiddlewareMessages_Location& input)
{
  if (!input.map_name)
    return convert_error("Location::map_name must not be null.");

  if (input.yaw_available)
  {
    return messages::Location(
      std::string(input.map_name),
      {input.x, input.y},
      input.yaw);
  }

  return messages::Location(std::string(input.map_name), {input.x, input.y});
}

//==============================================================================
std::optional<messages::Waypoint> convert(
  const MiddlewareMessages_Waypoint& input)
{
  auto loc = convert(input.location);
  if (!loc.has_value())
    return convert_error("failed to convert Waypoint::location.");

  if (input.wait_until_available)
  {
    auto wait_until = convert(input.wait_until);
    if (!wait_until.has_value())
      return convert_error("failed to convert Waypoint::wait_until.");

    return messages::Waypoint(
      static_cast<std::size_t>(input.index),
      loc.value(),
      wait_until.value());
  }

  return messages::Waypoint(
    static_cast<std::size_t>(input.index),
    loc.value());
}

//==============================================================================
std::optional<messages::RobotMode> convert(
  const MiddlewareMessages_RobotMode& input)
{
  using Mode = messages::RobotMode::Mode;
  Mode m;
  switch (input.mode)
  {
    case MiddlewareMessages_RobotMode_Constants_Idle:
      m = Mode::Idle;
      break;
    case MiddlewareMessages_RobotMode_Constants_Charging:
      m = Mode::Charging;
      break;
    case MiddlewareMessages_RobotMode_Constants_Moving:
      m = Mode::Moving;
      break;
    case MiddlewareMessages_RobotMode_Constants_Paused:
      m = Mode::Paused;
      break;
    case MiddlewareMessages_RobotMode_Constants_Waiting:
      m = Mode::Waiting;
      break;
    case MiddlewareMessages_RobotMode_Constants_Emergency:
      m = Mode::Emergency;
      break;
    case MiddlewareMessages_RobotMode_Constants_Docking:
      m = Mode::Docking;
      break;
    case MiddlewareMessages_RobotMode_Constants_Error:
      m = Mode::Error;
      break;
    case MiddlewareMessages_RobotMode_Constants_Undefined:
      m = Mode::Undefined;
      break;
    case MiddlewareMessages_RobotMode_Constants_Custom:
      m = Mode::Custom;
      break;
    default:
      m = Mode::Undefined;
  }

  if (!input.info)
    return convert_error("failed to convert RobotMode::info.");

  std::string info(input.info);
  if (!info.empty())
    return messages::RobotMode(m, std::string(input.info));

  return messages::RobotMode(m);
}

//==============================================================================
std::optional<messages::PauseRequest> convert(
  const MiddlewareMessages_PauseRequest& input)
{
  if (!input.robot_name)
    return convert_error("PauseRequest::robot_name must not be null.");

  return messages::PauseRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id));
}

//==============================================================================
std::optional<messages::ResumeRequest> convert(
  const MiddlewareMessages_ResumeRequest& input)
{
  if (!input.robot_name)
    return convert_error("ResumeRequest::robot_name must not be null.");

  return messages::ResumeRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id));
}

//==============================================================================
std::optional<messages::DockRequest> convert(
  const MiddlewareMessages_DockRequest& input)
{
  if (!input.robot_name)
    return convert_error("DockRequest::robot_name must not be null.");

  if (!input.dock_name)
    return convert_error("DockRequest::dock_name must not be null.");

  return messages::DockRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id),
    std::string(input.dock_name));
}

//==============================================================================
std::optional<messages::NavigationRequest> convert(
  const MiddlewareMessages_NavigationRequest& input)
{
  if (!input.robot_name)
    return convert_error("NavigationRequest::robot_name must not be null.");

  std::vector<messages::Waypoint> path;
  for (uint32_t i = 0; i < input.path._length; ++i)
  {
    auto wp = convert(input.path._buffer[i]);
    if (!wp.has_value())
    {
      std::stringstream ss;
      ss << "failed to convert Waypoint " << i
         << " in NavigationRequest::path.";
      return convert_error(ss.str());
    }
    path.push_back(wp.value());
  }

  return messages::NavigationRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id),
    std::move(path));
}

//==============================================================================
std::optional<messages::RelocalizationRequest> convert(
  const MiddlewareMessages_RelocalizationRequest& input)
{
  if (!input.robot_name)
    return convert_error("RelocalizationRequest::robot_name must not be null.");

  auto loc = convert(input.location);
  if (!loc.has_value())
    return convert_error("failed to convert RelocalizationRequest::location.");

  return messages::RelocalizationRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id),
    loc.value(),
    static_cast<std::size_t>(input.last_visited_waypoint_index));
}

//==============================================================================
std::optional<messages::RobotState> convert(
  const MiddlewareMessages_RobotState& input)
{
  auto time = convert(input.time);
  if (!time.has_value())
    return convert_error("failed to convert RobotState::time.");

  if (!input.name)
    return convert_error("RobotState::name must not be null.");

  if (!input.model)
    return convert_error("RobotState::model must not be null.");

  auto mode = convert(input.mode);
  if (!mode.has_value())
    return convert_error("failed to convert RobotState::mode.");

  auto loc = convert(input.location);
  if (!loc.has_value())
    return convert_error("failed to convert RobotState::location.");

  std::optional<TaskId> task_id = std::nullopt;
  if (input.task_id_available)
    task_id = static_cast<TaskId>(input.task_id);

  return messages::RobotState(
    time.value(),
    std::string(input.name),
    std::string(input.model),
    task_id,
    mode.value(),
    input.battery_percent,
    loc.value(),
    static_cast<std::size_t>(input.target_path_index));
}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet
