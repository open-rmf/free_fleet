/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <dds/dds.h>

#include "../dds_utils/common.hpp"

#include "message_utils.hpp"

namespace free_fleet {
namespace messages {

void convert(const RobotMode& _input, FreeFleetData_RobotMode& _output)
{
  // Consequently, free fleet robot modes need to be ordered similarly as 
  // RMF robot modes.
  _output.mode = _input.mode;
}

void convert(const FreeFleetData_RobotMode& _input, RobotMode& _output)
{
  // Consequently, free fleet robot modes need to be ordered similarly as 
  // RMF robot modes.
  _output.mode = _input.mode;
}

void convert(const Location& _input, FreeFleetData_Location& _output)
{
  _output.sec = _input.sec;
  _output.nanosec = _input.nanosec;
  _output.x = _input.x;
  _output.y = _input.y;
  _output.yaw = _input.yaw;
  _output.level_name = common::dds_string_alloc_and_copy(_input.level_name);
}

void convert(const FreeFleetData_Location& _input, Location& _output)
{
  _output.sec = _input.sec;
  _output.nanosec = _input.nanosec;
  _output.x = _input.x;
  _output.y = _input.y;
  _output.yaw = _input.yaw;
  _output.level_name = std::string(_input.level_name);
}

void convert(const RobotState& _input, FreeFleetData_RobotState& _output)
{
  _output.name = common::dds_string_alloc_and_copy(_input.name);
  _output.model = common::dds_string_alloc_and_copy(_input.model);
  _output.task_id = common::dds_string_alloc_and_copy(_input.task_id);
  convert(_input.mode, _output.mode);
  _output.battery_percent = _input.battery_percent;
  convert(_input.location, _output.location);

  size_t path_length = _input.path.size();
  _output.path._maximum = static_cast<uint32_t>(path_length);
  _output.path._length = static_cast<uint32_t>(path_length);
  _output.path._buffer = 
      FreeFleetData_RobotState_path_seq_allocbuf(path_length);
  _output.path._release = false;
  for (size_t i = 0; i < path_length; ++i)
    convert(_input.path[i], _output.path._buffer[i]);
}

void convert(const FreeFleetData_RobotState& _input, RobotState& _output)
{
  _output.name = std::string(_input.name);
  _output.model = std::string(_input.model);
  _output.task_id = std::string(_input.task_id);
  convert(_input.mode, _output.mode);
  _output.battery_percent = _input.battery_percent;
  convert(_input.location, _output.location);

  _output.path.clear();
  for (uint32_t i = 0; i < _input.path._length; ++i)
  {
    Location tmp;
    convert(_input.path._buffer[i], tmp);
    _output.path.push_back(tmp);
  }
}


void convert(const ModeParameter& _input, FreeFleetData_ModeParameter& _output)
{
  _output.name = common::dds_string_alloc_and_copy(_input.name);
  _output.value = common::dds_string_alloc_and_copy(_input.value);
}

void convert(const FreeFleetData_ModeParameter& _input, ModeParameter& _output)
{
  _output.name = std::string(_input.name);
  _output.value = std::string(_input.value);
}

void convert(const ModeRequest& _input, FreeFleetData_ModeRequest& _output)
{
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.robot_name = common::dds_string_alloc_and_copy(_input.robot_name);
  convert(_input.mode, _output.mode);
  _output.task_id = common::dds_string_alloc_and_copy(_input.task_id);

  size_t mode_parameter_num = _input.parameters.size();
  _output.parameters._maximum = static_cast<uint32_t>(mode_parameter_num);
  _output.parameters._length = static_cast<uint32_t>(mode_parameter_num);
  _output.parameters._buffer = 
      FreeFleetData_ModeRequest_parameters_seq_allocbuf(mode_parameter_num);
  for (size_t i = 0; i < mode_parameter_num; ++i)
    convert(_input.parameters[i], _output.parameters._buffer[i]);
}

void convert(const FreeFleetData_ModeRequest& _input, ModeRequest& _output)
{
  _output.fleet_name = std::string(_input.fleet_name);
  _output.robot_name = std::string(_input.robot_name);
  convert(_input.mode, _output.mode);
  _output.task_id = std::string(_input.task_id);

  _output.parameters.clear();
  for (uint32_t i = 0; i < _input.parameters._length; ++i)
  {
    ModeParameter tmp;
    convert(_input.parameters._buffer[i], tmp);
    _output.parameters.push_back(tmp);
  }
}

void convert(const PathRequest& _input, FreeFleetData_PathRequest& _output)
{
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.robot_name = common::dds_string_alloc_and_copy(_input.robot_name);

  size_t path_length = _input.path.size();
  _output.path._maximum = static_cast<uint32_t>(path_length);
  _output.path._length = static_cast<uint32_t>(path_length);
  _output.path._buffer = 
      FreeFleetData_PathRequest_path_seq_allocbuf(path_length);
  for (size_t i = 0; i < path_length; ++i)
    convert(_input.path[i], _output.path._buffer[i]);

  _output.task_id = common::dds_string_alloc_and_copy(_input.task_id);
}

void convert(const FreeFleetData_PathRequest& _input, PathRequest& _output)
{
  _output.fleet_name = std::string(_input.fleet_name);
  _output.robot_name = std::string(_input.robot_name);

  _output.path.clear();
  for (uint32_t i = 0; i < _input.path._length; ++i)
  {
    Location tmp;
    convert(_input.path._buffer[i], tmp);
    _output.path.push_back(tmp);
  }

  _output.task_id = std::string(_input.task_id);
}

void convert(
    const DestinationRequest& _input, 
    FreeFleetData_DestinationRequest& _output)
{
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.robot_name = common::dds_string_alloc_and_copy(_input.robot_name);
  convert(_input.destination, _output.destination);
  _output.task_id = common::dds_string_alloc_and_copy(_input.task_id);
}

void convert(
    const FreeFleetData_DestinationRequest& _input,
    DestinationRequest& _output)
{
  _output.fleet_name = std::string(_input.fleet_name);
  _output.robot_name = std::string(_input.robot_name);
  convert(_input.destination, _output.destination);
  _output.task_id = std::string(_input.task_id);
}

} // namespace messages
} // namespace free_fleet
