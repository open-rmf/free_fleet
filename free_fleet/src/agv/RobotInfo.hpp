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

#ifndef SRC__AGV__ROBOTINFO_HPP
#define SRC__AGV__ROBOTINFO_HPP

#include <string>
#include <memory>
#include <unordered_map>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <free_fleet/Manager.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/RobotState.hpp>

#include "../requests/RequestInfo.hpp"

namespace free_fleet {
namespace agv {

class RobotInfo
{
public:

  using SharedPtr = std::shared_ptr<RobotInfo>;

  /// These are different states of tracking the robot throughout the navigation
  /// graph. From top to bottom of descending preference.
  enum class TrackingState : uint8_t
  {
    OnWaypoint,
    OnLane,
    TowardsWaypoint,
    Lost
  };

  /// Gets the name of the robot.
  std::string name() const;

  /// Gets the model of the robot.
  std::string model() const;

  /// Gets a const reference to the most recent state.
  const messages::RobotState& state() const;

  /// The time stamp of when the last state update happened.
  rmf_traffic::Time last_updated() const;

  /// The time stamp of when the robot was first discovered.
  rmf_traffic::Time first_found() const;

  /// Allocates this task to this robot.
  ///
  /// \param[in] new_request_info
  ///   Pointer to a request.
  void allocate_task(
    const std::shared_ptr<requests::RequestInfo>& new_request_info);

  /// Update the internal robot handler with the newest state.
  ///
  /// \param[in] new_state
  ///   The most recent incoming state from the robot.
  ///
  /// \param[in] time_now
  ///   The current time stamp relative to the fleet manager.
  void update_state(
    const messages::RobotState& new_state,
    rmf_traffic::Time time_now);

private:
  friend class free_fleet::Manager;

  RobotInfo(
    const messages::RobotState& state,
    std::shared_ptr<rmf_traffic::agv::Graph> graph,
    rmf_traffic::Time time_now);

  void _track_and_update(const messages::RobotState& new_state);

  /// Finds the nearest waypoint in the graph to the location and its distance
  /// from it in meters.
  std::pair<rmf_traffic::agv::Graph::Waypoint*, double> _find_nearest_waypoint(
    const Eigen::Vector2d& coordinates) const;

  /// Finds the nearest lane in the graph by normal distance, and its distance
  /// away in meters.
  std::pair<rmf_traffic::agv::Graph::Lane*, double> _find_nearest_lane(
    const Eigen::Vector2d& coordinates) const;

  /// Checks whether the normal projection of a point onto a lane is within the
  /// entry and exit.
  bool _is_within_lane(
    rmf_traffic::agv::Graph::Lane* lane,
    const Eigen::Vector2d& coordinates) const;

  std::string _name;
  std::string _model;

  std::unordered_map<uint32_t, std::shared_ptr<requests::RequestInfo>>
    _allocated_requests;

  rmf_traffic::Time _first_found;
  rmf_traffic::Time _last_updated;
  rmf_utils::optional<messages::RobotState> _state;

  std::shared_ptr<rmf_traffic::agv::Graph> _graph;

  TrackingState _tracking_state;
  std::size_t _tracking_index;
};

} // namespace agv
} // namespace free_fleet

#endif // SRC__AGV__ROBOTINFO_HPP
