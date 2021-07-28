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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__LOCATION_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__LOCATION_HPP

#include <string>
#include <cstdint>
#include <optional>

#include <Eigen/Geometry>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class Location
{
public:

  /// Constructor
  ///
  /// \param[in] map_name
  ///   The name of the map this Location is referenced in. A
  ///   std::invalid_argument will be thrown if this is empty.
  ///
  /// \param[in] coordinates
  ///   The coordinates (x, y) of this Location in meters.
  Location(
    const std::string& map_name,
    Eigen::Vector2d coordinates);

  /// Constructor
  ///
  /// \param[in] map_name
  ///   The name of the map this Location is referenced in. A
  ///   std::invalid_argument will be thrown if this is empty.
  ///
  /// \param[in] coordinates
  ///   The coordinates (x, y) of this Location in meters.
  ///
  /// \param[in] yaw
  ///   The yaw of this Location in radians.
  Location(
    const std::string& map_name,
    Eigen::Vector2d coordinates,
    double yaw);

  /// Gets the current map name.
  const std::string& map_name() const;

  /// Gets the coordinates of this Location (x, y) in meters.
  const Eigen::Vector2d& coordinates() const;

  /// Gets the yaw in radians of this location. If the yaw is not required, a
  /// nullopt will be returned.
  std::optional<double> yaw() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//=============================================================================
/// Comparing operators
bool operator==(const Location& lhs, const Location& rhs);

bool operator!=(const Location& lhs, const Location& rhs);

//=============================================================================
} // namespace messages
} // namespace free_fleet

# endif // INCLUDE__FREE_FLEET__MESSAGES__LOCATION_HPP
