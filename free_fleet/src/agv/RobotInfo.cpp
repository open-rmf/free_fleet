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

#include <free_fleet/agv/RobotInfo.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include "internal_RobotInfo.hpp"
#include "../RequestInfo.hpp"

namespace free_fleet {
namespace agv {

//==============================================================================
void RobotInfo::Implementation::allocate_task(
  const std::shared_ptr<requests::BaseRequestInfo>& new_request_info)
{
  bool request_registered =
    allocated_requests.insert({
      new_request_info->id(),
      new_request_info}).second;
  if (!request_registered)
  {
    std::cerr << "[Warning]: Attempted to allocate new task with existing task "
      << "ID [" << new_request_info->id() << "]" << std::endl;
  }
}

//==============================================================================
void RobotInfo::Implementation::update_state(
  const messages::RobotState& new_state,
  rmf_traffic::Time time_now)
{
  if (name != new_state.name)
    return;
  track_and_update(new_state);
  last_updated = time_now;
}

//==============================================================================
void RobotInfo::Implementation::track_and_update(
  const messages::RobotState& new_state)
{
  const Eigen::Vector2d curr_loc = {new_state.location.x, new_state.location.y};

  // If the robot is not performing any task, we first check if it is near a
  // previous waypoint, before going through the entire navigation graph
  if (new_state.task_id == 0)
  {
    track_without_task_id(curr_loc);
  }
  else
  {
    const uint32_t task_id = new_state.task_id;
    auto it = allocated_requests.find(task_id);
    if (it == allocated_requests.end())
    {
      std::cerr << "[Warning]: Robot [" << name << "] task ID [" << task_id
        << "] was not given through the manager, due to lack of information, "
        "this robot is considered LOST." << std::endl;
      track_without_task_id(curr_loc);
      state = new_state;
      return;
    }
    assert(it->second);
    auto request = it->second;
    auto request_type = request->request_type();

    using RequestType = requests::BaseRequestInfo::RequestType;

    if (request_type == RequestType::PauseRequest ||
      request_type == RequestType::ResumeRequest ||
      request_type == RequestType::DockRequest)
    {
      // Pause, Resume and Dock requests will mainly be for changing perceived
      // locations. Therefore it should not effect tracking.
      track_without_task_id(curr_loc);
    }
    else if (request_type == RequestType::RelocalizationRequest)
    {
      // We will first check for the last visited waypoint.
      std::shared_ptr<requests::RequestInfo<messages::RelocalizationRequest>>
        reloc_req =
          std::dynamic_pointer_cast<
            requests::RequestInfo<messages::RelocalizationRequest>>(request);

      if (is_near_waypoint(
        reloc_req->request().last_visited_waypoint_index, curr_loc))
      {
        // We are very close to the provided waypoint
        tracking_state = TrackingState::OnWaypoint;
        tracking_index = reloc_req->request().last_visited_waypoint_index;
      }
      else
      {
        // We will try to localize just based on the waypoints in the navigation
        // graph, otherwise it is considered to be lost.
        track_without_task_id(curr_loc);
      }
    }
    else if (request_type == RequestType::NavigationRequest)
    {
      // We will use the target waypoints in the navigation request as
      // additional information to help with tracking.
      std::shared_ptr<requests::RequestInfo<messages::NavigationRequest>>
        nav_req =
          std::dynamic_pointer_cast<
            requests::RequestInfo<messages::NavigationRequest>>(request);
      const std::size_t next_wp_index =
        nav_req->request().path[new_state.path_target_index].index;

      rmf_utils::optional<std::size_t> prev_wp_index = rmf_utils::nullopt;
      const rmf_traffic::agv::Graph::Lane* curr_lane = nullptr;
      if (new_state.path_target_index != 0)
      {
        prev_wp_index =
          nav_req->request().path[new_state.path_target_index - 1].index;
        curr_lane = graph->lane_from(prev_wp_index.value(), next_wp_index);
      }

      if (is_near_waypoint(next_wp_index, curr_loc))
      {
        // The robot has reached the next waypoint on this navigation request.
        tracking_state = TrackingState::OnWaypoint;
        tracking_index = next_wp_index;
      }
      else if (prev_wp_index.has_value())
      {
        if (is_near_waypoint(prev_wp_index.value(), curr_loc))
        {
          // The robot is still in the previous waypoint on this navigation
          // request.
          tracking_state = TrackingState::OnWaypoint;
          tracking_index = prev_wp_index.value();
        }
        else if (curr_lane && is_within_lane(curr_lane, curr_loc))
        {
          tracking_state = TrackingState::OnLane;
          tracking_index = curr_lane->index();

          double dist_to_lane = distance_to_lane(curr_lane, curr_loc);
          if (dist_to_lane > lane_dist_threshold)
          {
            std::cerr << "[Warning]: Robot [" << name << "] is " << dist_to_lane
              << "m away from the lane center." << std::endl;
          }
        }
      }
      else
      {
        std::cerr << "[Warning]: Robot [" << name << "] is far away from the "
          "next waypoint, and there is no path from the previous "
          "waypoint, it is LOST until a clearer state comes in." << std::endl;
        tracking_state = TrackingState::Lost;
      }
    }
    else
    {
      // We have no other TrackingState.
    }
  }

  state = new_state;
}

//==============================================================================
void RobotInfo::Implementation::track_without_task_id(
  const Eigen::Vector2d& current_location)
{
  if (tracking_state == TrackingState::OnWaypoint &&
    is_near_waypoint(tracking_index, current_location))
  {
    // Do nothing, we are still on the same waypoint.
    return;
  }

  auto nearest_wp = find_nearest_waypoint(current_location);
  if (nearest_wp.second < waypoint_dist_threshold)
  {
    tracking_state = TrackingState::OnWaypoint;
    tracking_index = nearest_wp.first->index();
    return;
  }

  std::cerr << "[Warning]: Robot [" << name << "] has no task and is far away "
    "from any waypoints, it has diverged from the navigation graph "
    "and is LOST." << std::endl;
  tracking_state = TrackingState::Lost;
}

//==============================================================================
double RobotInfo::Implementation::distance_to_lane(
  const rmf_traffic::agv::Graph::Lane* lane,
  const Eigen::Vector2d& coordinates) const
{
  const Eigen::Vector2d entry_loc =
    graph->get_waypoint(lane->entry().waypoint_index()).get_location();
  const Eigen::Vector2d exit_loc =
    graph->get_waypoint(lane->exit().waypoint_index()).get_location();
  const Eigen::Vector3d entry_p = {entry_loc[0], entry_loc[1], 1.0};
  const Eigen::Vector3d exit_p = {exit_loc[0], exit_loc[1], 1.0};

  Eigen::Vector3d line_coef = entry_p.cross(exit_p);
  line_coef = line_coef / line_coef[2];

  const double dist =
    abs(line_coef[0] * coordinates[0] + line_coef[1] * coordinates[1] + 
    line_coef[2]) / sqrt(pow(line_coef[0], 2.0) + pow(line_coef[1], 2.0));
  return dist;
}

//=============================================================================
auto RobotInfo::Implementation::find_nearest_waypoint(
  const Eigen::Vector2d& coordinates) const
  -> std::pair<const rmf_traffic::agv::Graph::Waypoint*, double>
{
  const rmf_traffic::agv::Graph::Waypoint* nearest_wp = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < graph->num_waypoints(); ++i)
  {
    auto& wp = graph->get_waypoint(i);
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
auto RobotInfo::Implementation::find_nearest_lane(
  const Eigen::Vector2d& coordinates) const
  -> std::pair<const rmf_traffic::agv::Graph::Lane*, double>
{
  const rmf_traffic::agv::Graph::Lane* nearest_lane = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < graph->num_lanes(); ++i)
  {
    auto& l = graph->get_lane(i);

    // The normal formed by the given coordinates must also lie within the entry
    // and exit, to be considered
    if (!is_within_lane(&l, coordinates))
      continue;

    const double dist = distance_to_lane(&l, coordinates);
    if (dist < nearest_dist)
    {
      nearest_lane = &l;
      nearest_dist = dist;
    }
  }
  return std::make_pair(nearest_lane, nearest_dist);
}

//==============================================================================
bool RobotInfo::Implementation::is_within_lane(
  const rmf_traffic::agv::Graph::Lane* lane,
  const Eigen::Vector2d& coordinates) const
{
  const Eigen::Vector2d p0 =
    graph->get_waypoint(lane->entry().waypoint_index()).get_location();
  const Eigen::Vector2d p1 =
    graph->get_waypoint(lane->exit().waypoint_index()).get_location();
  const double lane_length = (p1 - p0).norm();
  
  const Eigen::Vector2d pn = (p1 - p0) / lane_length;
  const Eigen::Vector2d p_l = coordinates - p0;
  const double p_l_projection = p_l.dot(pn);

  if (p_l_projection >= 0.0 && p_l_projection <= lane_length)
    return true;
  return false;
}

//==============================================================================
bool RobotInfo::Implementation::is_near_waypoint(
  std::size_t waypoint_index,
  const Eigen::Vector2d& coordinates) const
{
  const Eigen::Vector2d p = graph->get_waypoint(waypoint_index).get_location();

  if ((p - coordinates).norm() < waypoint_dist_threshold)
    return true;
  return false;
}

//==============================================================================
RobotInfo::RobotInfo()
{}

//==============================================================================
std::string RobotInfo::name() const
{
  return _pimpl->name;
}

//==============================================================================
std::string RobotInfo::model() const
{
  return _pimpl->model;
}

//==============================================================================
rmf_traffic::Time RobotInfo::first_found() const
{
  return _pimpl->first_found;
}

//==============================================================================
const messages::RobotState& RobotInfo::state() const
{
  return _pimpl->state.value();
}

//==============================================================================
rmf_traffic::Time RobotInfo::last_updated() const
{
  return _pimpl->last_updated;
}

//==============================================================================
std::shared_ptr<const rmf_traffic::agv::Graph> RobotInfo::graph() const
{
  return _pimpl->graph;
}

//==============================================================================
auto RobotInfo::tracking_estimation() const 
  -> std::pair<TrackingState, std::size_t>
{
  return std::make_pair(_pimpl->tracking_state, _pimpl->tracking_index);
}

//==============================================================================

} // namespace agv
} // namespace free_fleet
