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

#ifndef INCLUDE__FREE_FLEET__AGV__STATUSHANDLE_HPP
#define INCLUDE__FREE_FLEET__AGV__STATUSHANDLE_HPP

#include <Eigen/Geometry>

#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/RobotMode.hpp>

namespace free_fleet {
namespace agv {

class StatusHandle
{
public:

  /// Obtain the current location of the robot.
  ///
  /// \return
  ///   Location of the robot including the time that it was sampled.
  virtual messages::Location location() = 0;

  /// Obtain the current mode of the robot.
  ///
  /// \return
  ///   Mode of the robot.
  virtual messages::RobotMode mode() = 0;

  /// Obtain the current battery percentage of the robot.
  ///
  /// \return
  ///   Battery percentage of the robot.
  virtual double battery_percent() = 0;

  /// Virtual destructor
  ~StatusHandle() = default;
};

} // namespace agv
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__AGV__STATUSHANDLE_HPP
