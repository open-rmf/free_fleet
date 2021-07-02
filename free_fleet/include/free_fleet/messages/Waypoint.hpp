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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__WAYPOINT_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__WAYPOINT_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <free_fleet/messages/Location.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class Waypoint
{
public:

  /// Constructor
  ///
  /// \param[in] index
  ///   The index of this waypoint on the navigation graph.
  ///
  /// \param[in] location
  ///   The Location of this waypoint.
  Waypoint(
    std::size_t index,
    const Location& location);

  /// Constructor
  ///
  /// \param[in] index
  ///   The index of this waypoint on the navigation graph.
  ///
  /// \param[in] location
  ///   The Location of this waypoint.
  ///
  /// \param[in] wait_until 
  ///   The time that the robot is expected to wait until on this waypoint
  ///   before proceeding.
  Waypoint(
    std::size_t index,
    const Location& location,
    rmf_traffic::Time wait_until);

  /// Gets the index of this waypoint.
  std::size_t index() const;

  /// Gets the location of this waypoint.
  const Location& location() const;

  /// Gets the time stamp that the robot is expected to wait until on the
  /// waypoint before proceeding. If the robot is expected to move on
  /// immediately, this will be a nullopt.
  std::optional<rmf_traffic::Time> wait_until() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Comparing operators.
bool operator==(const Waypoint& lhs, const Waypoint& rhs);

bool operator!=(const Waypoint& lhs, const Waypoint& rhs);

//==============================================================================
} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__WAYPOINT_HPP
