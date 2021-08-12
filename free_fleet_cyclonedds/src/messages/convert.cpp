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

#include <free_fleet/Types.hpp>

#include "convert.hpp"

namespace free_fleet {
namespace cyclonedds {

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
MiddlewareMessages_Time convert(const rmf_traffic::Time& input)
{
  MiddlewareMessages_Time output;
  output.sec =
    static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(
      input.time_since_epoch()).count());
  output.nanosec =
    static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      input.time_since_epoch()).count() % 1000000000);
  return output;
}

//==============================================================================
MiddlewareMessages_Location convert(const messages::Location& input)
{
  MiddlewareMessages_Location output;
  output.map_name = dds_string_alloc_and_copy(input.map_name());
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
MiddlewareMessages_Waypoint convert(const messages::Waypoint& input)
{
  MiddlewareMessages_Waypoint output;
  output.index = static_cast<uint32_t>(input.index());
  output.location = convert(input.location());
  output.wait_until_available = false;

  if (input.wait_until().has_value())
  {
    output.wait_until_available = true;
    output.wait_until = convert(input.wait_until().value());
  }
  return output;
}

//==============================================================================
MiddlewareMessages_RobotMode convert(const messages::RobotMode& input)
{
  using Mode = messages::RobotMode::Mode;
    
  MiddlewareMessages_RobotMode output;
  switch(input.mode())
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
  return output;
}

//==============================================================================
MiddlewareMessages_PauseRequest convert(const messages::PauseRequest& input)
{
  MiddlewareMessages_PauseRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());
  output.task_id = static_cast<uint32_t>(input.task_id());
  return output;
}

//==============================================================================
MiddlewareMessages_ResumeRequest convert(const messages::ResumeRequest& input)
{
  MiddlewareMessages_ResumeRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());
  output.task_id = static_cast<uint32_t>(input.task_id());
  return output;
}

//==============================================================================
MiddlewareMessages_DockRequest convert(const messages::DockRequest& input)
{
  MiddlewareMessages_DockRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());
  output.task_id = static_cast<uint32_t>(input.task_id());
  output.dock_name = dds_string_alloc_and_copy(input.dock_name());
  return output;
}

//==============================================================================
MiddlewareMessages_NavigationRequest convert(
  const messages::NavigationRequest& input)
{
  MiddlewareMessages_NavigationRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());
  output.task_id = static_cast<uint32_t>(input.task_id());
  
  const std::size_t path_elem_num = input.path().size();
  output.path._maximum = static_cast<uint32_t>(path_elem_num);
  output.path._length = static_cast<uint32_t>(path_elem_num);
  output.path._buffer =
    dds_sequence_MiddlewareMessages_Waypoint_allocbuf(path_elem_num);
  for (std::size_t i = 0; i < path_elem_num; ++i)
    output.path._buffer[i] = convert(input.path()[i]);
  return output;
}

//==============================================================================
MiddlewareMessages_RelocalizationRequest convert(
  const messages::RelocalizationRequest& input)
{
  MiddlewareMessages_RelocalizationRequest output;
  output.robot_name = dds_string_alloc_and_copy(input.robot_name());
  output.task_id = static_cast<uint32_t>(input.task_id());
  output.location = convert(input.location());
  output.last_visited_waypoint_index =
    static_cast<uint32_t>(input.last_visited_waypoint_index());
  return output;
}

//==============================================================================
MiddlewareMessages_RobotState convert(const messages::RobotState& input)
{
  MiddlewareMessages_RobotState output;
  output.time = convert(input.time());
  output.name = dds_string_alloc_and_copy(input.name());
  output.model = dds_string_alloc_and_copy(input.model());
  
  output.task_id_available = false;
  if (input.task_id().has_value())
  {
    output.task_id_available = true;
    output.task_id = static_cast<uint32_t>(input.task_id().value());
  }

  output.mode = convert(input.mode());
  output.battery_percent = input.battery_percent();
  output.location  = convert(input.location());
  output.target_path_index = static_cast<uint32_t>(input.target_path_index());

  return output;
}

//==============================================================================
rmf_traffic::Time convert(const MiddlewareMessages_Time& input)
{
  auto output = rmf_traffic::Time() +
    std::chrono::seconds(input.sec) +
    std::chrono::nanoseconds(input.nanosec);
  return output;
}

//==============================================================================
messages::Location convert(const MiddlewareMessages_Location& input)
{
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
messages::Waypoint convert(const MiddlewareMessages_Waypoint& input)
{
  if (input.wait_until_available)
  {
    return messages::Waypoint(
      static_cast<std::size_t>(input.index),
      convert(input.location),
      convert(input.wait_until));
  }
  return messages::Waypoint(
    static_cast<std::size_t>(input.index),
    convert(input.location));
}

//==============================================================================
messages::RobotMode convert(const MiddlewareMessages_RobotMode& input)
{
  using Mode = messages::RobotMode::Mode;
  Mode m;
  switch(input.mode)
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

  if (input.info)
    return messages::RobotMode(m, std::string(input.info));
  
  return messages::RobotMode(m);
}

//==============================================================================
messages::PauseRequest convert(const MiddlewareMessages_PauseRequest& input)
{
  return messages::PauseRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id));
}

//==============================================================================
messages::ResumeRequest convert(const MiddlewareMessages_ResumeRequest& input)
{
  return messages::ResumeRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id));
}

//==============================================================================
messages::DockRequest convert(const MiddlewareMessages_DockRequest& input)
{
  return messages::DockRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id),
    std::string(input.dock_name));
}

//==============================================================================
messages::NavigationRequest convert(
  const MiddlewareMessages_NavigationRequest& input)
{
  std::vector<messages::Waypoint> path;
  for (uint32_t i = 0; i < input.path._length; ++i)
    path.push_back(convert(input.path._buffer[i]));

  return messages::NavigationRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id),
    std::move(path));
}

//==============================================================================
messages::RelocalizationRequest convert(
  const MiddlewareMessages_RelocalizationRequest& input)
{
  return messages::RelocalizationRequest(
    std::string(input.robot_name),
    static_cast<TaskId>(input.task_id),
    convert(input.location),
    static_cast<std::size_t>(input.last_visited_waypoint_index));
}

//==============================================================================
messages::RobotState convert(const MiddlewareMessages_RobotState& input)
{
  std::optional<TaskId> task_id = std::nullopt;
  if (input.task_id_available)
    task_id = static_cast<TaskId>(input.task_id);

  return messages::RobotState(
    convert(input.time),
    std::string(input.name),
    std::string(input.model),
    task_id,
    convert(input.mode),
    input.battery_percent,
    convert(input.location),
    static_cast<std::size_t>(input.target_path_index));
}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet
