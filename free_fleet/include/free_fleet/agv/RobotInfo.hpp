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

#ifndef INCLUDE__FREE_FLEET__AGV__ROBOTINFO_HPP
#define INCLUDE__FREE_FLEET__AGV__ROBOTINFO_HPP

#include <string>
#include <memory>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <free_fleet/messages/RobotState.hpp>

namespace free_fleet {
namespace agv {

class RobotInfo
{
public:

  /// Enum of different tracking states depending on how the robot has been
  /// navigating through the graph, as well as its goals.
  enum class TrackingState : uint8_t
  {
    /// Robot is in close proximity of a waypoint.
    OnWaypoint,

    /// Robot is on a lane between two waypoints.
    OnLane,

    /// Robot has diverged from the navigation graph, with no valid goal
    /// information.
    Lost
  };

  /// The name of the robot.
  std::string name() const;

  /// The model of the robot.
  std::string model() const;

  /// The time stamp that this RobotInfo was first initialized.
  rmf_traffic::Time first_found() const;

  /// Gets a const reference to the most recent state.
  const messages::RobotState& state() const;

  /// The time stamp that this RobotInfo was last updated with a state.
  rmf_traffic::Time last_updated() const;

  /// Navigation graph of the robot.
  std::shared_ptr<const rmf_traffic::agv::Graph> graph() const;

  /// Gets the most up to date tracking estimation of this robot within the
  /// navigation graph.
  ///
  /// \return
  ///   Pair consists of the tracking state, as well as the index of the
  ///   component that it is tracked to. If the tracking state is Lost, the
  ///   second value is meaningless.
  std::pair<TrackingState, std::size_t> tracking_estimation() const;

  class Implementation;
private:
  RobotInfo();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__AGV__ROBOTINFO_HPP
