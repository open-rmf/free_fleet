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
#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/ModeParameter.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include <rmf_traffic/agv/Graph.hpp>

#include "MiddlewareMessages.h"

namespace free_fleet {
namespace cyclonedds {

char* dds_string_alloc_and_copy(const std::string& str);

// Location

void convert(
  const messages::Location& input,
  MiddlewareMessages_Location& output);

void convert(
  const MiddlewareMessages_Location& input,
  messages::Location& output);

// Waypoint

void convert(
  const messages::Waypoint& input,
  MiddlewareMessages_Waypoint& output);

void convert(
  const MiddlewareMessages_Waypoint& input,
  messages::Waypoint& output);

// ModeParameter

void convert(
  const messages::ModeParameter& input,
  MiddlewareMessages_ModeParameter& output);

void convert(
  const MiddlewareMessages_ModeParameter& input,
  messages::ModeParameter& output);

// ModeRequest

void convert(
  const messages::ModeRequest& input,
  MiddlewareMessages_ModeRequest& output);

void convert(
  const MiddlewareMessages_ModeRequest& input,
  messages::ModeRequest& output);

// NavigationRequest

void convert(
  const messages::NavigationRequest& input,
  MiddlewareMessages_NavigationRequest& output);

void convert(
  const MiddlewareMessages_NavigationRequest& input,
  messages::NavigationRequest& output);

// RelocalizationRequest

void convert(
  const messages::RelocalizationRequest& input,
  MiddlewareMessages_RelocalizationRequest& output);

void convert(
  const MiddlewareMessages_RelocalizationRequest& input,
  messages::RelocalizationRequest& output);

// RobotMode

void convert(
  const messages::RobotMode& input,
  MiddlewareMessages_RobotMode& output);

void convert(
  const MiddlewareMessages_RobotMode& input,
  messages::RobotMode& output);

// RobotState

void convert(
  const messages::RobotState& input,
  MiddlewareMessages_RobotState& output);

void convert(
  const MiddlewareMessages_RobotState& input,
  messages::RobotState& output);

} // namespace cyclonedds
} // namespace free_fleet

#endif // SRC__MESSAGES__UTILS_HPP
