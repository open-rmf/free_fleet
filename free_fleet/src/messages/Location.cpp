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

#include <free_fleet/Console.hpp>
#include <free_fleet/messages/Location.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class Location::Implementation
{
public:

  std::string map_name;

  Eigen::Vector2d coordinates;

  std::optional<double> yaw;
};

//==============================================================================
Location::Location(
  const std::string& map_name,
  Eigen::Vector2d coordinates)
{
  std::string error_message;

  if (map_name.empty())
    error_message = "map_name in Location must not be empty.";

  if (!error_message.empty())
  {
    fferr << error_message << "\n";
    throw std::invalid_argument(error_message);
  }

  _pimpl = rmf_utils::make_impl<Implementation>(Implementation{
    map_name,
    coordinates,
    std::nullopt});
}

//==============================================================================
Location::Location(
  const std::string& map_name,
  Eigen::Vector2d coordinates,
  double yaw)
{
  std::string error_message;

  if (map_name.empty())
    error_message = "map_name in Location must not be empty.";

  if (!error_message.empty())
  {
    fferr << error_message << "\n";
    throw std::invalid_argument(error_message);
  }

  _pimpl = rmf_utils::make_impl<Implementation>(Implementation{
    map_name,
    coordinates,
    yaw});
}

//==============================================================================
const std::string& Location::map_name() const
{
  return _pimpl->map_name;
}

//==============================================================================
const Eigen::Vector2d& Location::coordinates() const
{
  return _pimpl->coordinates;
}

//==============================================================================
std::optional<double> Location::yaw() const
{
  return _pimpl->yaw;
}

//=============================================================================
bool operator==(const Location& lhs, const Location& rhs)
{
  if (lhs.map_name() == rhs.map_name() &&      
    (lhs.coordinates() - rhs.coordinates()).norm() < 1e-3)
  {
    if (lhs.yaw().has_value() && rhs.yaw().has_value() &&
      abs(lhs.yaw().value() - rhs.yaw().value()) < 1e-3)
      return true;
    else if(!lhs.yaw().has_value() && !rhs.yaw().has_value())
      return true;
  }
  return false;
}

//=============================================================================
bool operator!=(const Location& lhs, const Location& rhs)
{
  return !(lhs == rhs);
}

//==============================================================================
} // namespace messages
} // namespace free_fleet
