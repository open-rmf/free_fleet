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

#ifndef SRC__AGV__INTERNAL__ROBOTINFO_HPP
#define SRC__AGV__INTERNAL__ROBOTINFO_HPP

#include <iostream>

#include <rmf_utils/optional.hpp>

#include <free_fleet/agv/RobotInfo.hpp>

#include "../requests/RequestInfo.hpp"

namespace free_fleet {
namespace agv {

class RobotInfo::Implementation
{
public:

  std::string name;
  std::string model;
  rmf_traffic::Time first_found;
  rmf_traffic::Time last_updated;
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  std::size_t tracking_index;
  TrackingState tracking_state = TrackingState::Lost;
  rmf_utils::optional<messages::RobotState> state = rmf_utils::nullopt;

  std::unordered_map<uint32_t, std::shared_ptr<requests::RequestInfo>>
    allocated_requests;

  const double waypoint_dist_threshold = 0.5;
  const double lane_dist_threshold = 1.0;

  static std::shared_ptr<RobotInfo> make(
    const messages::RobotState& state,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    rmf_traffic::Time time_now)
  {
    auto make_error_fn = [](const std::string& error_msg)
    {
      std::cerr << error_msg << std::endl;
      return nullptr;
    };

    if (state.name.empty())
      return make_error_fn("Provided robot name in state must not be empty.");
    if (state.model.empty())
      return make_error_fn("Provided robot model in state must not be empty.");
    if (!graph)
      return make_error_fn("Provided traffic graph is invalid.");

    RobotInfo info;
    info._pimpl = rmf_utils::make_impl<Implementation>(Implementation());
    info._pimpl->name = state.name;
    info._pimpl->model = state.model;
    info._pimpl->first_found = time_now;
    info._pimpl->last_updated = time_now;
    info._pimpl->graph = std::move(graph);
    info._pimpl->track_and_update(state);
    return std::make_shared<RobotInfo>(std::move(info));
  }

  static Implementation& get(RobotInfo& robot_info)
  {
    return *robot_info._pimpl;
  }

  static const Implementation& get(const RobotInfo& robot_info)
  {
    return *robot_info._pimpl;
  }

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

  /// Tracks the robot using the new incoming state and prior information.
  void track_and_update(const messages::RobotState& new_state);

  /// Tracks the robot using only the new incoming state information.
  void track_without_task_id(const Eigen::Vector2d& new_state);

  /// Finds the normal distance of a point to a lane.
  double distance_to_lane(
    const rmf_traffic::agv::Graph::Lane* lane,
    const Eigen::Vector2d& coordinates) const;

  /// Finds the nearest waypoint in the graph to the location and its distance
  /// from it in meters.
  std::pair<const rmf_traffic::agv::Graph::Waypoint*, double>
    find_nearest_waypoint(const Eigen::Vector2d& coordinates) const;

  /// Finds the nearest lane in the graph by normal distance, and its distance
  /// away in meters.
  std::pair<const rmf_traffic::agv::Graph::Lane*, double> find_nearest_lane(
    const Eigen::Vector2d& coordinates) const;

  /// Checks whether the normal projection of a point onto a lane is within the
  /// entry and exit.
  bool is_within_lane(
    const rmf_traffic::agv::Graph::Lane* lane,
    const Eigen::Vector2d& coordinates) const;

  /// Checks whether the coordinates are within 0.5 meters of a given waypoint.
  bool is_near_waypoint(
    std::size_t waypoint_index,
    const Eigen::Vector2d& coordinates) const;
};

} // namespace agv
} // namespace free_fleet

#endif // SRC__AGV__INTERNAL__ROBOTINFO_HPP
