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
rmf_traffic::agv::Graph::Waypoint* find_nearest_waypoint(
  const std::shared_ptr<rmf_traffic::agv::Graph>& graph,
  const messages::Location& location)
{
  const Eigen::Vector2d p(location.x, location.y);
  rmf_traffic::agv::Graph::Waypoint* nearest_wp = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < graph->num_waypoints(); ++i)
  {
    auto& wp = graph->get_waypoint(i);
    const Eigen::Vector2d wp_p = wp.get_location();
    const double dist = (p - wp_p).norm();
    if (dist < nearest_dist)
    {
      nearest_wp = &wp;
      nearest_dist = dist;
    }
  }
  return nearest_wp;
}

//==============================================================================
RobotInfo::RobotInfo(
  const messages::RobotState& state,
  std::shared_ptr<rmf_traffic::agv::Graph> graph,
  rmf_traffic::Time time_now)
: _name(state.name),
  _model(state.model),
  _first_found(time_now),
  _last_updated(time_now),
  _state(state),
  _graph(std::move(graph))
{}

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
  return _state;
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
void RobotInfo::update_state(
  const messages::RobotState& new_state,
  rmf_traffic::Time time_now)
{
  if (_name != new_state.name)
    return;

  _state = new_state;
  _last_updated = time_now;

  // keep track of where it is in the navigation graph
  // have an instance of where the last waypoint was

  // This is ideally only done only once when a new robot is registered.
  // Checking through the graph is expensive.
  if (!_last_known_wp.has_value() && !_lane_occupied.has_value())
  {
    // _last_known_wp =
    //   find_nearest_waypoint(_graph, new_state.location)->index();
    // const auto possible_lanes = _graph->lanes_from(_last_known_wp);
    // _possible_next_waypoints = {};
    // for (const auto l : possible_lanes)
    // {
    //   _possible_next_waypoints.push_back(l.exit().waypoint_index());
    // }
    // return;

    // Check if the robot is heading some where, that should give us some hints
    if (!_state.path.empty())
    {
      // get the next waypoint from the intended path
      std::size_t next_wp = static_cast<std::size_t>(_state.path[0].index);
      const Eigen::Vector2d next_wp_loc =
        _graph->get_waypoint(next_wp).get_location();

      // get the lane that has this waypoint as the exit
      double nearest_dist = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < _graph->num_lanes(); ++i)
      {
        const auto l = _graph->get_lane(i);
        const std::size_t entry_index = l.entry().waypoint_index();
        const std::size_t exit_index = l.exit().waypoint_index();
        if (exit_index != next_wp)
          continue;

        const Eigen::Vector2d entry_loc =
          _graph->get_waypoint(entry_index).get_location();
        const Eigen::Vector2d exit_loc =
          _graph->get_waypoint(exit_index).get_location();
        const Eigen::Vector3d entry_p = {entry_loc[0], entry_loc[1], 1.0};
        const Eigen::Vector3d exit_p = {exit_loc[0], exit_loc[1], 1.0};
        
        // Get the line equation and normalize it
        Eigen::Vector3d line = entry_p.cross(exit_p);
        line = line / line[2];

        const double dist =
          abs(line[0] * next_wp_loc[0] + line[1] * next_wp_loc[1] + line[2]) /
          sqrt(pow(line[0], 2.0) + pow(line[1], 2.0));
        if (dist < nearest_dist)
        {
          nearest_dist = dist;
          _lane_occupied = i;
        }
      }
    }
  }
  // else if ()

  // if (!_last_known_wp.has_value())
  // {
  //   if (_lane_occupied.has_value())
  //   {
  //     // assign the last known waypoint as the entry
  //   }
  //   // check if it is on
  //   // find which waypoint it is nearest to
  // }

}

//==============================================================================
} // namespace agv
} // namespace free_fleet
