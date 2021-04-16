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

#include "utilities.hpp"

namespace free_fleet {

//==============================================================================
double distance_to_waypoint(
  const rmf_traffic::agv::Graph::Waypoint& waypoint,
  const Eigen::Vector2d& coordinates)
{
  const Eigen::Vector2d p = waypoint.get_location();
  return (p - coordinates).norm();
}

//==============================================================================
double distance_to_lane(
  const rmf_traffic::agv::Graph::Lane& lane,
  const rmf_traffic::agv::Graph& graph,
  const Eigen::Vector2d& coordinates)
{
  const Eigen::Vector2d entry_loc =
    graph.get_waypoint(lane.entry().waypoint_index()).get_location();
  const Eigen::Vector2d exit_loc =
    graph.get_waypoint(lane.exit().waypoint_index()).get_location();
  const Eigen::Vector3d entry_p = {entry_loc[0], entry_loc[1], 1.0};
  const Eigen::Vector3d exit_p = {exit_loc[0], exit_loc[1], 1.0};

  Eigen::Vector3d line_coef = entry_p.cross(exit_p);
  line_coef = line_coef / line_coef[2];

  const double dist =
    abs(line_coef[0] * coordinates[0] + line_coef[1] * coordinates[1] + 
    line_coef[2]) / sqrt(pow(line_coef[0], 2.0) + pow(line_coef[1], 2.0));
  return dist;
}

//==============================================================================
bool is_within_lane(
  const rmf_traffic::agv::Graph::Lane& lane,
  const rmf_traffic::agv::Graph& graph,
  const Eigen::Vector2d& coordinates)
{
  const Eigen::Vector2d p0 =
    graph.get_waypoint(lane.entry().waypoint_index()).get_location();
  const Eigen::Vector2d p1 =
    graph.get_waypoint(lane.exit().waypoint_index()).get_location();
  const double lane_length = (p1 - p0).norm();
  
  const Eigen::Vector2d pn = (p1 - p0) / lane_length;
  const Eigen::Vector2d p_l = coordinates - p0;
  const double p_l_projection = p_l.dot(pn);

  if (p_l_projection >= 0.0 && p_l_projection <= lane_length)
    return true;
  return false;
}

//==============================================================================
std::pair<const rmf_traffic::agv::Graph::Waypoint*, double>
  find_nearest_waypoint(
    const rmf_traffic::agv::Graph& graph,
    const Eigen::Vector2d& coordinates)
{
  const rmf_traffic::agv::Graph::Waypoint* nearest_wp = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
  {
    auto& wp = graph.get_waypoint(i);
    const Eigen::Vector2d wp_p = wp.get_location();
    const double dist = (coordinates - wp_p).norm();
    if (dist < nearest_dist)
    {
      nearest_wp = &wp;
      nearest_dist = dist;
    }
  }
  return std::make_pair(nearest_wp, nearest_dist);
}

//==============================================================================
std::pair<const rmf_traffic::agv::Graph::Lane*, double>
  find_nearest_lane(
    const rmf_traffic::agv::Graph& graph,
    const Eigen::Vector2d& coordinates)
{
  const rmf_traffic::agv::Graph::Lane* nearest_lane = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < graph.num_lanes(); ++i)
  {
    auto& l = graph.get_lane(i);

    // The normal formed by the given coordinates must also lie within the entry
    // and exit, to be considered
    if (!is_within_lane(l, graph, coordinates))
      continue;

    const double dist = distance_to_lane(l, graph, coordinates);
    if (dist < nearest_dist)
    {
      nearest_lane = &l;
      nearest_dist = dist;
    }
  }
  return std::make_pair(nearest_lane, nearest_dist);
}

//==============================================================================
} // namespace free_fleet
