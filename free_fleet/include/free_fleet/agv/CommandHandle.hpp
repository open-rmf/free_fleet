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

#ifndef INCLUDE__FREE_FLEET__AGV__COMMANDHANDLE_HPP
#define INCLUDE__FREE_FLEET__AGV__COMMANDHANDLE_HPP

#include <vector>
#include <functional>

namespace free_fleet {
namespace agv {

//==============================================================================
/// Implement this class to execute commands on the robot
class CommandHandle
{
public:

  /// Trigger this callback function when the follow_new_path request has been
  /// completed. It should only be triggered that one time and then discarded.
  using RequestCompleted = std::function<void()>;

  /// Trigger this callback function when the robot encounters an error 
  using ErrorEncountered = std::function<void()>;


  /// Have the robot follow a new path. If it was already following a path, then
  /// it should immediately switch over to this one.
  ///
  /// \param[in] waypoints
  ///   The sequence of waypoints to follow. When the robot arrives at a
  ///   waypoint in this sequence, it should wait at that waypoint until the
  ///   clock reaches the time() field of the waypoint. This is important
  ///   because the waypoint timing is used to avoid traffic conflicts with
  ///   other vehicles.
  ///
  /// \param[in] next_arrival_estimator
  ///   Give an estimate for how long the robot will take to reach the path
  ///   element of the specified index. You should still be calling
  ///   RobotUpdateHandle::update_position() even as you call this function.
  ///
  /// \param[in] path_finished_callback
  ///   Trigger this callback when the robot is done following the new path.
  ///   You do not need to trigger waypoint_arrival_callback when triggering
  ///   this one.
  virtual void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      ArrivalEstimator next_arrival_estimator,
      RequestCompleted path_finished_callback) = 0;

  /// Have the robot come to an immediate stop.
  virtual void stop() = 0;

  /// Virtual destructor
  virtual ~CommandHandle() = default;

};

} // namespace agv
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__AGV__COMMANDHANDLE_HPP
