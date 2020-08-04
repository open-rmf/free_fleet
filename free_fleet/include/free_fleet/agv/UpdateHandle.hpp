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

#ifndef INCLUDE__FREE_FLEET__AGV__UPDATEHANDLE_HPP
#define INCLUDE__FREE_FLEET__AGV__UPDATEHANDLE_HPP

#include <Eigen/Geometry>

#include <free_fleet/messages/RobotMode.hpp>

namespace free_fleet {
namespace agv {

class UpdateHandle
{
public:

  /// Obtain the current level name of the robot
  ///
  /// \return
  ///   Name of the current level as a string
  virtual std::string level_name() = 0;

  /// Obtain the current position of the robot
  ///
  /// \return
  ///   Pose of the robot in {x, y, yaw}
  virtual Eigen::Vector3d position() = 0;

  /// Obtain the current mode of the robot
  ///
  /// \return
  ///   Mode of the robot with info
  virtual messages::RobotMode mode() = 0;

  /// Virtual destructor
  virtual ~UpdateHandle() = default;
}

} // namespace agv
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__AGV__UPDATEHANDLE_HPP
