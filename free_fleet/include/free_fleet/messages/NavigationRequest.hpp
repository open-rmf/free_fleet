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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__NAVIGATIONREQUEST_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__NAVIGATIONREQUEST_HPP

#include <string>
#include <vector>

#include <rmf_utils/impl_ptr.hpp>

#include <free_fleet/Types.hpp>
#include <free_fleet/messages/Waypoint.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class NavigationRequest
{
public:

  /// Constructor
  ///
  /// \param[in] robot_name
  ///   The name of the robot this request is targeting. A std::invalid_argument
  ///   will be thrown if this is empty.
  ///
  /// \param[in] task_id
  ///   The task id associated with this request.
  ///
  /// \param[in] path
  ///   The desired path for this navigation request.
  NavigationRequest(
    const std::string& robot_name,
    TaskId task_id,
    std::vector<Waypoint> path);

  /// Gets the robot name.
  const std::string& robot_name() const;

  /// Gets the task id.
  TaskId task_id() const;

  /// Gets the desired path of this request.
  const std::vector<Waypoint>& path() const; 

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Comparing operators.
bool operator==(const NavigationRequest& lhs, const NavigationRequest& rhs);

bool operator!=(const NavigationRequest& lhs, const NavigationRequest& rhs);

//==============================================================================
} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__NAVIGATIONREQUEST_HPP
