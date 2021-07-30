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

#include <free_fleet/messages/Location.hpp>
// #include <free_fleet/messages/Waypoint.hpp>
// #include <free_fleet/messages/RobotMode.hpp>
// #include <free_fleet/messages/RobotState.hpp>
// #include <free_fleet/messages/ModeRequest.hpp>
// #include <free_fleet/messages/ModeParameter.hpp>
// #include <free_fleet/messages/NavigationRequest.hpp>
// #include <free_fleet/messages/RelocalizationRequest.hpp>

#include <rmf_traffic/Time.hpp>

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

///
void convert(const rmf_traffic::Time& input, MiddlewareMessages_Time& output);

///
void convert(const MiddlewareMessages_Time& input, rmf_traffic::Time& output);

///
void convert(
  const free_fleet::messages::Location& input,
  MiddlewareMessages_Location& output);

///
void convet(
  const MiddlewareMessages_Location& input,
  free_fleet::messages::Location& output);

// ///
// void convert(
//   const free_fleet::messages::Waypoint& input,
//   MiddlewareMessages_Waypoint& output);
// 
// ///
// void convert(
//   const MiddlewareMessages_Waypoint& input,
//   free_fleet::messages::Waypoint& output);
// 
// ///
// void convert(
//   const free_fleet::messages::RobotMode& input,
//   MiddlewareMessages_RobotMode& output);
// 
// ///
// void convert(
//   const MiddlewareMessages_RobotMode& input,
//   free_fleet::messages::RobotMode& output);
// 
// ///
// void convert(
//   const free_fleet::messages::PauseRequest& input,
//   MiddlewareMessages_PauseRequest& output);
// 
// ///
// void convert(
//   const MiddlewareMessages_PauseRequest& input,
//   free_fleet::messages::PauseRequest& output);
// 
// ///
// void convert(
//   const free_fleet::messages::ResumeRequest& input,
//   MiddlewareMessages_ResumeRequest& output);
// 
// ///
// void convert(
//   const MiddlewareMessages_ResumeRequest& input,
//   free_fleet::messages::ResumeRequest& output);
// 
// ///
// void convert(
//   const free_fleet::messages::DockRequest& input,
//   MiddlewareMessages_DockRequest& output);
// 
// ///
// void convert(
//   const MiddlewareMessages_DockRequest& input,
//   free_fleet::messages::DockRequest& output);
// 
// ///
// void convert(
//   const free_fleet::messages::NavigationRequest& input,
//   MiddlewareMessages_NavigationRequest& output);
// 
// ///
// void convert(
//   const MiddlewareMessages_NavigationRequest& input,
//   free_fleet::messages::NavigationRequest& output);
// 
// ///
// void convert(
//   const free_fleet::messages::RelocalizationRequest& input,
//   MiddlewareMessages_RelocalizationRequest& output);
// 
// ///
// void convet(
//   const MiddlewareMessages_RelocalizationRequest& input,
//   free_fleet::messages::RelocalizationRequest& output);
// 
// ///
// void convert(
//   const free_fleet::messages::RobotState& input,
//   MiddlewareMessages_RobotState& output);
// 
// ///
// void convert(
//   const MiddlewareMessages_RobotState& input,
//   MiddlewareMessages_RobotState& output);

} // namespace cyclonedds
} // namespace free_fleet

#endif // SRC__MESSAGES__CONVERT_HPP
