/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <free_fleet/messages/Waypoint.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class Waypoint::Implementation
{
public:
  
  std::size_t index;

  Location location;

  std::optional<rmf_traffic::Time> wait_until;
};

//==============================================================================
Waypoint::Waypoint(std::size_t index, const Location& location)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{
    index, location, std::nullopt}))
{}

//==============================================================================
Waypoint::Waypoint(
  std::size_t index,
  const Location& location,
  rmf_traffic::Time wait_until)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{
    index, location, wait_until}))
{}

//==============================================================================
std::size_t Waypoint::index() const
{
  return _pimpl->index;
}

//==============================================================================
const Location& Waypoint::location() const
{
  return _pimpl->location;
}

//==============================================================================
std::optional<rmf_traffic::Time> Waypoint::wait_until() const
{
  return _pimpl->wait_until;
}

//==============================================================================
bool operator==(const Waypoint& lhs, const Waypoint& rhs)
{
  if (lhs.index() == rhs.index() &&
    lhs.location() == rhs.location())
    return true;
  return false;
}

//==============================================================================
bool operator!=(const Waypoint& lhs, const Waypoint& rhs)
{
  return !(lhs == rhs);
}

//==============================================================================
} // namespace messages
} // namespace free_fleet
