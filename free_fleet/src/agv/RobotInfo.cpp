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
#include "../utilities/utilities.hpp"
#include "../requests/SimpleRequestInfo.hpp"

namespace free_fleet {
namespace agv {

//==============================================================================
void RobotInfo::Implementation::allocate_task(
  const std::shared_ptr<requests::RequestInfo>& new_request_info)
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
  // Robot is not doing any task at the moment.
  if (new_state.task_id == 0)
  {
    auto new_tracking_estimate = track_through_graph(new_state);
    tracking_state = new_tracking_estimate.first;
    tracking_index = new_tracking_estimate.second;
    state = new_state;
    return;
  }
  
  // Use the information from the current task to help track the robot.
  const uint32_t task_id = new_state.task_id;
  auto it = allocated_requests.find(task_id);

  // No such task exists
  if (it == allocated_requests.end())
  {
    std::cerr << "[Warning]: Robot [" << name << "] task ID [" << task_id
      << "] was not allocated through the manager." << std::endl;

    auto new_tracking_estimate = track_through_graph(new_state);
    tracking_state = new_tracking_estimate.first;
    tracking_index = new_tracking_estimate.second;
    state = new_state;
    return;
  }

  // Use the implemented track_robot function of each request type to get new
  // tracking estimates.
  assert(it->second);
  auto request = it->second;
  auto new_tracking_estimate = request->track_robot(*parent, new_state);

  tracking_state = new_tracking_estimate.first;
  tracking_index = new_tracking_estimate.second;
  state = new_state;
}

//==============================================================================
auto RobotInfo::Implementation::track_through_graph(
  const messages::RobotState& new_state) const
  -> std::pair<TrackingState, std::size_t>
{
  const Eigen::Vector2d curr_loc = {new_state.location.x, new_state.location.y};

  if (tracking_state == agv::RobotInfo::TrackingState::OnWaypoint)
  {
    double distance =
      distance_to_waypoint(graph->get_waypoint(tracking_index), curr_loc);
    if (distance < waypoint_dist_threshold)
      return std::make_pair(tracking_state, tracking_index);
  }

  auto nearest_wp = find_nearest_waypoint(*graph, curr_loc);
  if (nearest_wp.first && nearest_wp.second < waypoint_dist_threshold)
    return std::make_pair(
      agv::RobotInfo::TrackingState::OnWaypoint, nearest_wp.first->index());

  std::cerr << "[Warning]: Robot [" << name << "] has no task and "
    "is far away from any waypoints, it has diverged from the navigation graph "
    "and is LOST." << std::endl;
  return std::make_pair(
    agv::RobotInfo::TrackingState::Lost, tracking_index);
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
