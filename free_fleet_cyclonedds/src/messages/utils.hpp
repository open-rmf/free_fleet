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

#ifndef SRC__MESSAGES__UTILS_HPP
#define SRC__MESSAGES__UTILS_HPP

#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/ModeParameter.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>

#include <rmf_traffic/agv/Graph.hpp>

#include "MiddlewareMessages.h"

namespace free_fleet {
namespace cyclonedds {

// Location

void convert(const Location& input, MiddlewareMessages_Lane& output);

void convert(const MiddlewareMessages_Lane& input, Location& output);

// ModeParameter

void convert(
    const ModeParameter& input, 
    MiddlewareMessages_ModeParameter& output);

void convert(
    const MiddlewareMessages_ModeParameter& input,
    ModeParameter& output);

// NavigationRequest

void convert(
    const NavigationRequest& input, 
    MiddlewareMessages_NavigationRequest& output);

void convert(
    const MiddlewareMessages_NavigationRequest& input, 
    NavigationRequest& output);

// RobotMode

void convert(const RobotMode& input, MiddlewareMessages_RobotMode& output);

void convert(const MiddlewareMessages_RobotMode& input, RobotMode& output);

// RobotState

void convert(const RobotState& input, MiddlewareMessages_Robotstate& output);

void convert(const MiddlewareMessages_Robotstate& input, RobotState& output);

// Graph

void convert(
    const rmf_traffic::agv::Graph& input, 
    MiddlewareMessages_Graph& output);

void convert(
    const MiddlewareMessages_Graph& input, 
    rmf_traffic::agv::Graph& output);

} // namespace cyclonedds
} // namespace free_fleet

#endif // SRC__MESSAGES__UTILS_HPP
