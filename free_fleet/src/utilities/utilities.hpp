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

#ifndef SRC__UTILITIES__UTILITIES_HPP
#define SRC__UTILITIES__UTILITIES_HPP

#include <rmf_utils/optional.hpp>
#include <rmf_traffic/agv/Graph.hpp>

namespace free_fleet {

/// Finds the distance to a waypoint.
double distance_to_waypoint(
  const rmf_traffic::agv::Graph::Waypoint& waypoint,
  const Eigen::Vector2d& coordinates);

/// Finds the perpendicular distance to a lane.
double distance_to_lane(
  const rmf_traffic::agv::Graph::Lane& lane,
  const rmf_traffic::agv::Graph& graph,
  const Eigen::Vector2d& coordinates);

/// Returns whether the provided coordinates lies within the perpendicular
/// boundary of the start and end of the lane.
bool is_within_lane(
  const rmf_traffic::agv::Graph::Lane& lane,
  const rmf_traffic::agv::Graph& graph,
  const Eigen::Vector2d& coordinates);

/// Finds the nearest waypoint and returns a pointer to it, as well as the
/// distance. If there is no such waypoint, the first of the pair will be a
/// nullptr, and the second should be ignored.
std::pair<const rmf_traffic::agv::Graph::Waypoint*, double>
  find_nearest_waypoint(
    const rmf_traffic::agv::Graph& graph,
    const Eigen::Vector2d& coordinates);

/// Finds the nearest lane and returns a pointer to it, as well as the distance.
/// If there is no such lane, the first of the pair will be a nullptr, and the
/// second should be ignored.
std::pair<const rmf_traffic::agv::Graph::Lane*, double>
  find_nearest_lane(
    const rmf_traffic::agv::Graph& graph,
    const Eigen::Vector2d& coordinates);

} // namespace free_fleet

#endif // SRC__UTILITIES__UTILITIES_HPP
