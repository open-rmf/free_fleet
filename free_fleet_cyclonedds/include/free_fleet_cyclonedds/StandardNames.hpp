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

#ifndef INCLUDE__FREE_FLEET_CYCLONEDDS__STANDARDNAMES_HPP
#define INCLUDE__FREE_FLEET_CYCLONEDDS__STANDARDNAMES_HPP

namespace free_fleet {
namespace cyclonedds {

constexpr char Prefix[] = "free_fleet";
constexpr char StateTopicName[] = "robot_state";
constexpr char DockRequestTopicName[] = "dock_request";
constexpr char PauseRequestTopicName[] = "pause_request";
constexpr char ResumeRequestTopicName[] = "resume_request";
constexpr char NavigationRequestTopicName[] = "navigation_request";
constexpr char RelocalizationRequestTopicName[] = "relocalization_request";

//==============================================================================
/// Appends one or more additional string elements to the first passed in
/// argument.
///
/// \param[in] args
///   The string elements to be appended together.
///
/// \return
///   The appended strings.
template <typename... Args>
std::string append(Args const&... args)
{
  std::string result;
  int unpack[]{0, (result += std::to_string(args) + "/", 0)...};
  static_cast<void>(unpack);
  return result;
}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET_CYCLONEDDS__STANDARDNAMES_HPP 
