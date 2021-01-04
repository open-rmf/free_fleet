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

#include <iostream>

#include "RobotInfo.hpp"
#include "../requests/ModeRequestInfo.hpp"
#include "../requests/NavigationRequestInfo.hpp"
#include "../requests/RelocalizationRequestInfo.hpp"

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
  _graph(std::move(graph)),
  _tracking_state(TrackingState::Lost)
{
  _track_and_update(state);
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
auto RobotInfo::tracking_estimation() const 
  -> std::pair<TrackingState, std::size_t>
{
  return std::make_pair(_tracking_state, _tracking_index);
}

//==============================================================================
void RobotInfo::_track_and_update(const messages::RobotState& new_state)
{
  const Eigen::Vector2d curr_loc = {new_state.location.x, new_state.location.y};

  // If the robot is not performing any task, we first check if it is near a
  // previous waypoint, before going through the entire navigation graph
  if (new_state.task_id == 0)
  {
    _track_without_task_id(curr_loc);
  }
  else
  {
    std::cout << "task found" << std::endl;
    const uint32_t task_id = new_state.task_id;
    auto it = _allocated_requests.find(task_id);
    if (it == _allocated_requests.end())
    {
      // GAH NOT GOOD, No such task was given to this robot through the manager,
      // due to lack of information for this task, we will treat this as the
      // robot is not doing any task.
      // TODO(AA): Warn that this is happening.
      std::cout << "no such task!" << std::endl;
      _track_without_task_id(curr_loc);
    }
    assert(it->second);
    auto request = it->second;
    auto request_type = request->request_type();

    using RequestType = requests::RequestInfo::RequestType;

    if (request_type == RequestType::ModeRequest)
    {
      // Mode and Relocalization requests will mainly be for pausing, resuming,
      // and changing perceived locations. Therefore it should not affect
      // tracking.
      _track_without_task_id(curr_loc);
    }
    else if (request_type == RequestType::RelocalizationRequest)
    {
      // We will first check for the last visited waypoint.
      std::shared_ptr<requests::RelocalizationRequestInfo> reloc_req =
        std::dynamic_pointer_cast<requests::RelocalizationRequestInfo>(
          request);

      if (_is_near_waypoint(
        reloc_req->request().last_visited_waypoint_index, curr_loc))
      {
        // We are very close to the provided waypoint
        _tracking_state = TrackingState::OnWaypoint;
        _tracking_index = reloc_req->request().last_visited_waypoint_index;
      }
      else
      {
        // We will try to localize just based on the waypoints in the navigation
        // graph, otherwise it is considered to be lost.
        _track_without_task_id(curr_loc);
      }
    }
    else if (request_type == RequestType::NavigationRequest)
    {
      std::cout << "it is a nav request" << std::endl;
      // We will use the target waypoints in the navigation request as
      // additional information to help with tracking.
      std::shared_ptr<requests::NavigationRequestInfo> nav_req =
        std::dynamic_pointer_cast<requests::NavigationRequestInfo>(request);
      const std::size_t next_wp_index =
        nav_req->request().path[new_state.path_target_index].index;

      rmf_utils::optional<std::size_t> prev_wp_index = rmf_utils::nullopt;
      rmf_traffic::agv::Graph::Lane* curr_lane = nullptr;
      if (new_state.path_target_index - 1 >= 0)
      {
        prev_wp_index =
          nav_req->request().path[new_state.path_target_index - 1].index;
        std::cout << prev_wp_index.value() << std::endl;
        curr_lane = _graph->lane_from(prev_wp_index.value(), next_wp_index);
        std::cout << curr_lane->index() << std::endl;
      }

      if (_is_near_waypoint(next_wp_index, curr_loc))
      {
        std::cout << "It is near waypoint" << std::endl;
        // The robot has reached the next waypoint on this navigation request.
        _tracking_state = TrackingState::OnWaypoint;
        _tracking_index = next_wp_index;
      }
      else if (prev_wp_index.has_value())
      {
        if (_is_near_waypoint(prev_wp_index.value(), curr_loc))
        {
          // The robot is still in the previous waypoint on this navigation
          // request.
          _tracking_state = TrackingState::OnWaypoint;
          _tracking_index = prev_wp_index.value();
        }
        else if (curr_lane && _is_within_lane(curr_lane, curr_loc))
        {
          _tracking_state = TrackingState::OnLane;
          _tracking_index = curr_lane->index();

          // TODO(AA): Warn if it is far away from lane.
          // double dist_to_lane = _distance_to_lane(lane, curr_loc);
          // if (dist_to_lane > _lane_dist_threshold)
          // {
          //   // warn
          // }
        }
      }
    }
    else
    {
      // We have no other TrackingState.
    }
  }

  _state = new_state;
}

//==============================================================================
void RobotInfo::_track_without_task_id(const Eigen::Vector2d& current_location)
{
  if (_tracking_state == TrackingState::OnWaypoint &&
    _is_near_waypoint(_tracking_index, current_location))
  {
    // Do nothing, we are still on the same waypoint.
    return;
  }

  // Because we have no task, and we are quite far away from any
  // waypoints, we have diverged from the navigation graph and would be
  // considered quite lost.
  // TODO(AA): Warn that this is happening.
  auto nearest_wp = _find_nearest_waypoint(current_location);
  if (nearest_wp.second < _waypoint_dist_threshold)
  {
    _tracking_state = TrackingState::OnWaypoint;
    _tracking_index = nearest_wp.first->index();
    return;
  }

  // It is lost even by the best estimates.
  _tracking_state = TrackingState::Lost;
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

    const double dist = _distance_to_lane(&l, coordinates);
    if (dist < nearest_dist)
    {
      nearest_lane = &l;
      nearest_dist = dist;
    }
  }
  return std::make_pair(nearest_lane, nearest_dist);
}

//==============================================================================
double RobotInfo::_distance_to_lane(
  rmf_traffic::agv::Graph::Lane* lane,
  const Eigen::Vector2d& coordinates) const
{
  const Eigen::Vector2d entry_loc =
    _graph->get_waypoint(lane->entry().waypoint_index()).get_location();
  const Eigen::Vector2d exit_loc =
    _graph->get_waypoint(lane->exit().waypoint_index()).get_location();
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
bool RobotInfo::_is_near_waypoint(
  std::size_t waypoint_index,
  const Eigen::Vector2d& coordinates) const
{
  const Eigen::Vector2d p = _graph->get_waypoint(waypoint_index).get_location();

  if ((p - coordinates).norm() < _waypoint_dist_threshold)
    return true;
  return false;
}

//==============================================================================
} // namespace agv
} // namespace free_fleet
