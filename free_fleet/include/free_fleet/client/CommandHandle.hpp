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

#ifndef INCLUDE__FREE_FLEET__CLIENT__COMMANDHANDLE_HPP
#define INCLUDE__FREE_FLEET__CLIENT__COMMANDHANDLE_HPP

#include <vector>
#include <functional>

#include <free_fleet/messages/Waypoint.hpp>

namespace free_fleet {
namespace client {

//==============================================================================
/// Implement this class to execute commands on the robot.
class CommandHandle
{
public:

  /// Trigger this callback function when the go_to_new_destination request has
  /// been completed. It should only be triggered that one time and then
  /// discarded.
  using RequestCompleted = std::function<void()>;

  /// Have the robot attempt to relocalize remotely.
  ///
  /// \param[in] location
  ///   The location in the frame of the robot for relocalization. 
  ///
  /// \param[in] relocalization_finished_callback
  ///   Trigger this callback when the robot is done with relocalization.
  virtual void relocalize(
    const free_fleet::messages::Location& location,
    RequestCompleted relocalization_finished_callback) = 0;

  /// Have the robot follow a new path. If it was already following a path, then
  /// it should immediately switch over to this one.
  ///
  /// \param[in] waypoints
  ///   The sequence of waypoints to follow. When the robot arrives at a
  ///   waypoint in this sequence, it should wait at that waypoint until the
  ///   clock reaches the time described in the waypoint. This is important
  ///   because the waypoint timing is used to avoid traffic conflicts with
  ///   other vehicles.
  ///
  /// \param[in] path_finished_callback
  ///   Trigger this callback when the robot is done following the new path.
  virtual void follow_new_path(
      const std::vector<free_fleet::messages::Waypoint>& waypoints,
      RequestCompleted path_finished_callback) = 0;

  /// Have the robot come to an immediate stop.
  virtual void stop(RequestCompleted stopped_callback) = 0;

  /// Have the robot resume performing its task before it was stopped.
  virtual void resume(RequestCompleted resumed_callback) = 0;

  /// Have the robot begin a pre-defined docking procedure. Implement this
  /// function as a no-op if your robots do not perform docking procedures.
  ///
  /// \param[in] dock_name
  ///   The predefined name of the docking procedure to use.
  ///
  /// \param[in] docking_finished_callback
  ///   Trigger this callback when the docking is finished.
  virtual void dock(
      const std::string& dock_name,
      RequestCompleted docking_finished_callback) = 0;

  /// Virtual destructor
  virtual ~CommandHandle() = default;
};

} // namespace client
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__CLIENT__COMMANDHANDLE_HPP
