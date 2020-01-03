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

#include <free_fleet/Location.hpp>
#include <free_fleet/RobotMode.hpp>
#include <free_fleet/RobotState.hpp>
#include <free_fleet/ModeRequest.hpp>
#include <free_fleet/PathRequest.hpp>
#include <free_fleet/DestinationRequest.hpp>

#include "FreeFleet.h"

#include "message_utils.hpp"

namespace free_fleet
{
namespace messages
{

void convert(const RobotMode& _input, FreeFleetData_RobotMode& _output)
{
  switch(_input.mode)
  {
    case RobotMode::MODE_CHARGING:
      _output.mode = FreeFleetData_RobotMode_Constants_MODE_CHARGING;
      break;
    case RobotMode::MODE_MOVING:
      _output.mode = FreeFleetData_RobotMode_Constants_MODE_MOVING;
      break;
    case RobotMode::MODE_PAUSED:
      _output.mode = FreeFleetData_RobotMode_Constants_MODE_PAUSED;
      break;
    case RobotMode::MODE_WAITING:
      _output.mode = FreeFleetData_RobotMode_Constants_MODE_WAITING;
      break;
    case RobotMode::MODE_EMERGENCY:
      _output.mode = FreeFleetData_RobotMode_Constants_MODE_EMERGENCY;
      break;
    case RobotMode::MODE_GOING_HOME:
      _output.mode = FreeFleetData_RobotMode_Constants_MODE_GOING_HOME;
      break;
    default:
      _output.mode = FreeFleetData_RobotMode_Constants_MODE_IDLE;
  }
}

void convert(const Location& _input, FreeFleetData_Location& _output)
{
  
}

} // namespace messages
} // namespace free_fleet
