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

#ifndef SRC__MESSAGES__CONVERT_HPP
#define SRC__MESSAGES__CONVERT_HPP

#include <optional>

#include <rmf_traffic/Time.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include "MiddlewareMessages.h"

namespace free_fleet {
namespace cyclonedds {

/// Allocates and copies the string into the DDS message char array.
///
/// \param[in] str
///   The string to be copied.
///
/// \return
///   The char pointer which has memory allocated and string copied into.
char* dds_string_alloc_and_copy(const std::string& str);

//==============================================================================
/// From free_fleet::messages to DDS messages, nullopt if the conversion failed.
std::optional<MiddlewareMessages_Time> convert(const rmf_traffic::Time& input);

std::optional<MiddlewareMessages_Location> convert(
  const messages::Location& input);

std::optional<MiddlewareMessages_Waypoint> convert(
  const free_fleet::messages::Waypoint& input);

std::optional<MiddlewareMessages_RobotMode> convert(
  const free_fleet::messages::RobotMode& input);

std::optional<MiddlewareMessages_PauseRequest> convert(
  const free_fleet::messages::PauseRequest& input);

std::optional<MiddlewareMessages_ResumeRequest> convert(
  const free_fleet::messages::ResumeRequest& input);

std::optional<MiddlewareMessages_DockRequest> convert(
  const free_fleet::messages::DockRequest& input);

std::optional<MiddlewareMessages_NavigationRequest> convert(
  const free_fleet::messages::NavigationRequest& input);

std::optional<MiddlewareMessages_RelocalizationRequest> convert(
  const free_fleet::messages::RelocalizationRequest& input);

std::optional<MiddlewareMessages_RobotState> convert(
  const free_fleet::messages::RobotState& input);

//==============================================================================
/// From DDS messages to free_fleet::messages, nullopt if the conversion failed.
std::optional<rmf_traffic::Time> convert(const MiddlewareMessages_Time& input);

std::optional<messages::Location> convert(
  const MiddlewareMessages_Location& input);

std::optional<messages::Waypoint> convert(
  const MiddlewareMessages_Waypoint& input);

std::optional<messages::RobotMode> convert(
  const MiddlewareMessages_RobotMode& input);

std::optional<messages::PauseRequest> convert(
  const MiddlewareMessages_PauseRequest& input);

std::optional<messages::ResumeRequest> convert(
  const MiddlewareMessages_ResumeRequest& input);

std::optional<messages::DockRequest> convert(
  const MiddlewareMessages_DockRequest& input);

std::optional<messages::NavigationRequest> convert(
  const MiddlewareMessages_NavigationRequest& input);

std::optional<messages::RelocalizationRequest> convert(
  const MiddlewareMessages_RelocalizationRequest& input);

std::optional<messages::RobotState> convert(
  const MiddlewareMessages_RobotState& input);

} // namespace cyclonedds
} // namespace free_fleet

#endif // SRC__MESSAGES__CONVERT_HPP
