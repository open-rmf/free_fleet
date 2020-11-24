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

  const Eigen::Vector2d curr_loc(_state.location.x, _state.location.y);
  double dist_thresh = 0.5;

  // if there is no intended path to be followed, we can safely assume that the
  // robot is near a waypoint, or is lost and unintentionally occupying a lane,
  // which we will not track
  if (_state.path.empty())
  {
    // If the robot was previously on a lane, check if it has reached the exit
    if (_lane_occupied.has_value())
    {
      std::size_t next_wp =
        _graph->get_lane(_lane_occupied.value()).exit().waypoint_index();
      const Eigen::Vector2d next_wp_loc =
        _graph->get_waypoint(next_wp).get_location();
      const double dist = (curr_loc - next_wp_loc).norm();
      if (dist < dist_thresh)
      {
        _last_known_wp = next_wp;
      }
      else
      {
        // Warning, robot is lost or is drifting away from the navigation graph
        // Give the printout information of its last known waypoint or occupied
        // lane
      }
    }
    // Robot was previously known to be on this waypoint, check if it is still
    // around the waypoint area
    else if (_last_known_wp.has_value())
    {
      const Eigen::Vector2d last_wp_loc =
        _graph->get_waypoint(_last_known_wp.value()).get_location();
      const double dist = (curr_loc - last_wp_loc).norm();
      if (dist > dist_thresh)
      {
        // Warning, robot is lost or is drifting away from the navigation graph
        // Give the printout information of its last known waypoint
      }
    }
    // Robot was not previously known to be on any waypoint, nor occupying any
    // lanes in the navigation graph, we can only use the state's location to
    // estimate where the robot is
    else
    {
      // Find the closest waypoints, and set them as potential last known
      // waypoints
      const Eigen::Vector2d loc(_state.location.x, _state.location.y);
      rmf_traffic::agv::Graph::Waypoint* nearest_wp = nullptr;
      double nearest_dist = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < _graph->num_waypoints(); ++i)
      {
        auto& wp = _graph->get_waypoint(i);
        const Eigen::Vector2d wp_loc = wp.get_location();
        const double dist = (loc - wp_loc).norm();
        if (dist < nearest_dist)
        {
          nearest_wp = &wp;
          nearest_dist = dist;
        }
      }
      // Ideally the closest waypoint is within threshold to be considered that
      // the robot is sitting on it
      if (nearest_dist < dist_thresh)
      {
        _last_known_wp = nearest_wp->index();
      }
      else
      {
        // Warning, robot is lost, waiting for it to arrive at any waypoint,
        // before tracking can resume.
      }
    }

    // Without any path in state, the lane occupied should always be empty.
    _lane_occupied = rmf_utils::nullopt;
  }
  // We can use the path to get some hints
  else
  {
    // If there was a previous lane, we check if it has already reached the exit
    if (_lane_occupied.has_value())
    {
      const std::size_t exit_wp =
        _graph->get_lane(_lane_occupied.value()).exit().waypoint_index();

      // If the exit waypoint is no longer on the next index on the path,
      // it might either have reached the waypoint, or the robot is following a
      // new path.
      if (static_cast<std::size_t>(_state.path[0].index) != exit_wp)
      {
        // nothing much has changed.
      }
      else
      {
        const Eigen::Vector2d exit_loc =
          _graph->get_waypoint(exit_wp).get_location();
        const double dist_to_exit = (curr_loc - exit_loc).norm();
        if (dist_to_exit < dist_thresh)
        {
          _last_known_wp = exit_wp;
          // check if it is also occupying the lane
        }
      }
      
    }
    else if (_last_known_wp.has_value())
    {

    }
    else
    {

    }
  }

  // keep track of where it is in the navigation graph
  // have an instance of where the last waypoint was

  // This is ideally only done only once when a new robot is registered.
  // Checking through the graph is expensive.
  // if (!_lane_occupied.has_value() && !_last_known_wp.has_value())
  // {
  //   // Check if the robot is heading some where, that should give us some hints
  //   if (!_state.path.empty())
  //   {
  //     // get the next waypoint from the intended path
  //     std::size_t next_wp = static_cast<std::size_t>(_state.path[0].index);
  //     const Eigen::Vector2d next_wp_loc =
  //       _graph->get_waypoint(next_wp).get_location();

  //     // get the lane that has this waypoint as the exit
  //     double nearest_dist = std::numeric_limits<double>::infinity();
  //     for (std::size_t i = 0; i < _graph->num_lanes(); ++i)
  //     {
  //       const auto l = _graph->get_lane(i);
  //       const std::size_t entry_index = l.entry().waypoint_index();
  //       const std::size_t exit_index = l.exit().waypoint_index();
  //       if (exit_index != next_wp)
  //         continue;

  //       const Eigen::Vector2d entry_loc =
  //         _graph->get_waypoint(entry_index).get_location();
  //       const Eigen::Vector2d exit_loc =
  //         _graph->get_waypoint(exit_index).get_location();
  //       const Eigen::Vector3d entry_p = {entry_loc[0], entry_loc[1], 1.0};
  //       const Eigen::Vector3d exit_p = {exit_loc[0], exit_loc[1], 1.0};
        
  //       // Get the line equation and normalize it
  //       Eigen::Vector3d line = entry_p.cross(exit_p);
  //       line = line / line[2];

  //       const double dist =
  //         abs(line[0] * next_wp_loc[0] + line[1] * next_wp_loc[1] + line[2]) /
  //         sqrt(pow(line[0], 2.0) + pow(line[1], 2.0));
  //       if (dist < nearest_dist)
  //       {
  //         nearest_dist = dist;
  //         _lane_occupied = i;
  //         _last_known_wp = entry_index;
  //       }
  //     }
  //   }
  //   // It is not going anywhere, we only have its location to use as estimation.
  //   // This is probably the worst case.
  //   else
  //   {
  //     const Eigen::Vector2d p(_state.location.x, _state.location.y);
  //     rmf_traffic::agv::Graph::Waypoint* nearest_wp = nullptr;
  //     double nearest_dist = std::numeric_limits<double>::infinity();
  //     for (std::size_t i = 0; i < _graph->num_waypoints(); ++i)
  //     {
  //       auto& wp = _graph->get_waypoint(i);
  //       const Eigen::Vector2d wp_p = wp.get_location();
  //       const double dist = (p - wp_p).norm();
  //       if (dist < nearest_dist)
  //       {
  //         nearest_wp = &wp;
  //         nearest_dist = dist;
  //       }
  //     }


  //   }
  // }
  // This will be the usual case, where we have been able to keep track of the
  // where the robot is.
  // else if (_last_known_wp.
  // {

  // }
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
