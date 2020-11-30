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

#include "RobotInfo.hpp"

namespace free_fleet {
namespace agv {

//==============================================================================
RobotInfo::RobotInfo(
  const messages::RobotState& state,
  std::shared_ptr<rmf_traffic::agv::Graph> graph,
  rmf_traffic::Time time_now)
: _name(state.name),
  _model(state.model),
  _first_found(time_now),
  _last_updated(time_now),
  _state(rmf_utils::nullopt),
  _graph(std::move(graph))
{
  _track(state);
}

//==============================================================================
std::string RobotInfo::name() const
{
  return _name;
}

//==============================================================================
std::string RobotInfo::model() const
{
  return _model;
}

//==============================================================================
const messages::RobotState& RobotInfo::state() const
{
  return _state.value();
}

//==============================================================================
rmf_traffic::Time RobotInfo::last_updated() const
{
  return _last_updated;
}

//==============================================================================
rmf_traffic::Time RobotInfo::first_found() const
{
  return _first_found;
}

//==============================================================================
void RobotInfo::allocate_task(
  const std::shared_ptr<requests::RequestInfo>& new_request_info)
{
  _allocated_requests[new_request_info->id()] = new_request_info;
}

//==============================================================================
void RobotInfo::update_state(
  const messages::RobotState& new_state,
  rmf_traffic::Time time_now)
{
  if (_name != new_state.name)
    return;
  _track_and_update(new_state);
  _last_updated = time_now;
}

//==============================================================================
void RobotInfo::_track_and_update(const messages::RobotState& new_state)
{
  // If the robot is not performing any task, we first check if it is near a
  // previous waypoint, before going throught the entire navigation graph
  if (new_state.task_id == 0)
  {

  }
}

//==============================================================================
std::pair<rmf_traffic::agv::Graph::Waypoint*, double>
  RobotInfo::_find_nearest_waypoint(const Eigen::Vector2d& coordinates) const
{
  rmf_traffic::agv::Graph::Waypoint* nearest_wp = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < _graph->num_waypoints(); ++i)
  {
    auto& wp = _graph->get_waypoint(i);
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
std::pair<rmf_traffic::agv::Graph::Lane*, double>
  RobotInfo::_find_nearest_lane(const Eigen::Vector2d& coordinates) const
{
  rmf_traffic::agv::Graph::Lane* nearest_lane = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < _graph->num_lanes(); ++i)
  {
    auto& l = _graph->get_lane(i);

    // The normal formed by the given coordinates must also lie within the entry
    // and exit, to be considered
    if (!_is_within_lane(&l, coordinates))
      continue;

    const std::size_t entry_index = l.entry().waypoint_index();
    const std::size_t exit_index = l.exit().waypoint_index();

    const Eigen::Vector2d entry_loc =
      _graph->get_waypoint(entry_index).get_location();
    const Eigen::Vector2d exit_loc =
      _graph->get_waypoint(exit_index).get_location();
    const Eigen::Vector3d entry_p = {entry_loc[0], entry_loc[1], 1.0};
    const Eigen::Vector3d exit_p = {exit_loc[0], exit_loc[1], 1.0};
    
    // Get the line equation and normalize it
    Eigen::Vector3d line_coef = entry_p.cross(exit_p);
    line_coef = line_coef / line_coef[2];

    const double dist =
      abs(line_coef[0] * coordinates[0] + line_coef[1] * coordinates[1] + 
      line_coef[2]) / sqrt(pow(line_coef[0], 2.0) + pow(line_coef[1], 2.0));
    if (dist < nearest_dist)
    {
      nearest_lane = &l;
      nearest_dist = dist;
    }
  }
  return std::make_pair(nearest_lane, nearest_dist);
}

//==============================================================================
bool RobotInfo::_is_within_lane(
  rmf_traffic::agv::Graph::Lane* lane,
  const Eigen::Vector2d& coordinates) const
{
  const Eigen::Vector2d p0 =
    _graph->get_waypoint(lane->entry().waypoint_index()).get_location();
  const Eigen::Vector2d p1 =
    _graph->get_waypoint(lane->exit().waypoint_index()).get_location();
  const double lane_length = (p1 - p0).norm();
  
  const Eigen::Vector2d pn = (p1 - p0) / lane_length;
  const Eigen::Vector2d p_l = coordinates - p0;
  const double p_l_projection = p_l.dot(pn);

  if (p_l_projection >= 0.0 && p_l_projection <= lane_length)
    return true;
  return false;
}

//==============================================================================
} // namespace agv
} // namespace free_fleet
