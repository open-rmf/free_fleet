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
  _graph(std::move(graph))
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
void RobotInfo::_track_without_task_id(
  const messages::RobotState& new_state)
{
  const Eigen::Vector2d curr_loc = {new_state.location.x, new_state.location.y};

  if (_tracking_state == TrackingState::OnWaypoint)
  {
    if (_is_near_waypoint(_tracking_index, curr_loc))
    {
      // Do nothing, we are still on the same waypoint
    }
    else
    {
      // Because we have no task, and we are quite far away from any
      // waypoints, we have diverged from the navigation graph and would be
      // considered lost
      // TODO(AA): Warn that this is happening.
      _tracking_state = TrackingState::Lost;
    }
  }
  else if (_tracking_state == TrackingState::OnLane)
  {
    auto& lane = _graph->get_lane(_tracking_index);
    const std::size_t exit_index = lane.exit().waypoint_index();

    if (_is_near_waypoint(exit_index, curr_loc))
    {
      // It is very close to the exit waypoint, we consider it tracked to
      // that waypoint.
      _tracking_state = TrackingState::OnWaypoint;
      _tracking_index = exit_index;
    }
    else
    {
      if (_is_within_lane(&lane, curr_loc))
      {
        // The robot is still within its lane, we will keep it that way
      }
      else
      {
        auto nearest_wp_pair = _find_nearest_waypoint(curr_loc);
        if (nearest_wp_pair.second < _waypoint_dist_threshold)
        {
          // The robot seems to be lost but it has managed to get near to a
          // waypoint different from the exit waypoint of the lane
          // TODO(AA): Warn that this is happening.
          _tracking_state = TrackingState::OnWaypoint;
          _tracking_index = nearest_wp_pair.first->index();
        }
        else
        {
          // It is no longer on its lane, nor anywhere near any waypoints.
          // TODO(AA): Warn that this is happening.
          _tracking_state = TrackingState::Lost;
        }
      }
    }
  }
  else if (_tracking_state == TrackingState::TowardsWaypoint)
  {
    auto& target_wp = _graph->get_waypoint(_tracking_index);
    const Eigen::Vector2d target_loc = target_wp.get_location();

    if (_is_near_waypoint(_tracking_index, curr_loc))
    {
      // The robot is very near to its target waypoint, we change the tracking
      // state but keep the same tracking index
      _tracking_state = TrackingState::OnWaypoint;
    }
    else
    {
      // The robot is not near its target waypoint yet, however due to the
      // lack of a task, we will keep the same tracking state and target
      // waypoint
    }
  }
  else
  {
    // Robot is currently lost, all we can do is to check if it is currently
    // near any waypoints, or lanes
    auto nearest_wp_pair = _find_nearest_waypoint(curr_loc);
    if (nearest_wp_pair.first &&
      nearest_wp_pair.second < _waypoint_dist_threshold)
    {
      _tracking_state = TrackingState::OnWaypoint;
      _tracking_index = nearest_wp_pair.first->index();
      return;
    }

    auto nearest_lane_pair = _find_nearest_lane(curr_loc);
    if (nearest_lane_pair.first &&
      nearest_lane_pair.second < _lane_dist_threshold &&
      _is_within_lane(nearest_lane_pair.first, curr_loc))
    {
      _tracking_state = TrackingState::OnLane;
      _tracking_index = nearest_lane_pair.first->index();
      return;
    }

    // Otherwise the robot is still lost
    _tracking_state = TrackingState::Lost;
  }
}

//==============================================================================
void RobotInfo::_track_and_update(const messages::RobotState& new_state)
{
  const Eigen::Vector2d curr_loc = {new_state.location.x, new_state.location.y};

  // If the robot is not performing any task, we first check if it is near a
  // previous waypoint, before going through the entire navigation graph
  if (new_state.task_id == 0)
  {
    _track_without_task_id(new_state);
  }
  else
  {
    const uint32_t task_id = new_state.task_id;
    auto it = _allocated_requests.find(task_id);
    if (it == _allocated_requests.end())
    {
      // GAH NOT GOOD, No such task was given to this robot through the manager,
      // due to lack of information for this task, we will treat this as the
      // robot is not doing any task.
      // TODO(AA): Warn that this is happening.
      _track_without_task_id(new_state);
    }
    assert(it->second);
    auto request = it->second;
    auto request_type = request->request_type();

    using RequestType = requests::RequestInfo::RequestType;

    if (_tracking_state == TrackingState::OnWaypoint)
    {
      if (request_type == RequestType::ModeRequest)
      {
        // Mode and Relocalization requests will mainly be for pausing,
        // resuming, and changing perceived locations. Therefore it
        // should not affect tracking.
        _track_without_task_id(new_state);
      }
      else if (request_type == RequestType::NavigationRequest)
      {
        // We will use the target waypoints in the navigation request as
        // additional information to help with tracking.
        std::shared_ptr<requests::NavigationRequestInfo> nav_req =
          std::dynamic_pointer_cast<requests::NavigationRequestInfo>(
            request);
        const std::size_t next_wp_index =
          nav_req->request().path[new_state.path_target_index];
        
        // Check if it has reached the target waypoint
        if (_is_near_waypoint(next_wp_index, curr_loc))
        {
          _tracking_state = TrackingState::OnWaypoint;
          _tracking_index = next_wp_index;
          return;
        }

        // Prioritize obeying the previous tracking state
        if (_tracking_state == TrackingState::OnWaypoint)
        {
          if (_is_near_waypoint(_tracking_index, curr_loc))
          {
            // Robot is still very close to previous tracked waypoint, we do
            // nothing.
          }
          else
          {
            const auto* lane =
              _graph->lane_from(_tracking_index, next_wp_index);
            if (!lane)
            {
              // This is the best estimation we can make
              _tracking_state = TrackingState::TowardsWaypoint;
              _tracking_index = next_wp_index;
            }
            else
            {
              const std::size_t exit_wp_index = lane->exit().waypoint_index();
              if (_is_near_waypoint(exit_wp_index, curr_loc))
              {
                // Robot has reached the end of the lane
                _tracking_state = TrackingState::OnWaypoint;
                _tracking_index = exit_wp_index;
              }
              else if (_is_within_lane(lane, curr_loc))
              {
                // Robot is on the lane
                _tracking_state = TrackingState::OnLane;
                _tracking_index = lane->index();
              }
              else
              {
                // This is the best estimation we can make
                _tracking_state = TrackingState::TowardsWaypoint;
                _tracking_index = next_wp_index;
              }
            }
          }
        }
        else if (_tracking_state == TrackingState::OnLane)
        {
          const auto& tracked_lane = _graph->get_lane(_tracking_index);
          if (tracked_lane.exit().waypoint_index() == next_wp_index)
          {
            // Our previous tracking is good, 
            const std::size_t exit_wp_index = lane.exit().waypoint_index();
            if (_is_near_waypoint(exit_wp_index, curr_loc))
            {
              // Robot has reached the end of the lane
              _tracking_state = TrackingState::OnWaypoint;
              _tracking_index = exit_wp_index;
            }
            else if (_is_within_lane(&tracked_lane, curr_loc))
            {
              // Robot is still on the same lane, we don't need to do anything
            }
            else
            {
              // This is the best estimation we can make
              _tracking_state = TrackingState::TowardsWaypoint;
              _tracking_index = next_wp_index;
            }
          }
          else
          {
            // We were somehow tracking a different lane previously, this is not
            // great, we will start tracking from scratch again
            _tracking_state = TrackingState::TowardsWaypoint;
            _tracking_index = next_wp_index;
          }
        }
        else if (_tracking_state == TrackingState::TowardsWaypoint)
        {
          if (_tracking_index == next_wp_index)
          {
            if (_is_near_waypoint(_tracking_index, curr_loc))
            {
              // We have reached the target waypoint, this should allow us to
              // reset for better tracking from this waypoint onwards.
              _tracking_state = TrackingState::OnWaypoint;
            }
            else
            {
              if (new_state.path_target_index != 0)
              {
                // Check to see if we are on the intended lane
                const std::size_t prev_wp_index =
                  nav_req->request().path[new_state.path_target_index - 1];
                const auto* supposed_lane =
                  _graph->lane_from(prev_wp_index, next_wp_index);
                if (supposed_lane && _is_within_lane(supposed_lane, curr_loc))
                {
                  _tracking_state = TrackingState::OnLane;
                  _tracking_index = supposed_lane->index();
                }
              }
              else
              {
                // The robot has not yet reached its targetted waypoint, and is
                // not strictly within the desired lane, we will keep its
                // tracking state to be TowardsWaypoint
              }
            }
          }
          else
          {
            // The previous tracking was wayyyy off
            _tracking_state = TrackingState::TowardsWaypoint;
            _tracking_index = next_wp_index;
          }
        }
        else
        {
          if (_is_near_waypoint(next_wp_index, curr_loc))
          {
            // The robot was initially lost, but have found its way to the next
            // target waypoint on its intended path
            _tracking_state = TrackingState::OnWaypoint;
            _tracking_index = next_wp_index;
          }
          else
          {
            if (new_state.path_target_index != 0)
            {
              // Try to infer the lane information from the path
              const std::size_t prev_wp_index =
                nav_req->request().path[new_state.path_target_index - 1];
              const auto* supposed_lane =
                _graph->lane_from(prev_wp_index, next_wp_index);
              if (supposed_lane && _is_within_lane(supposed_lane, curr_loc))
              {
                _tracking_state = TrackingState::OnLane;
                _tracking_index = supposed_lane->index();
              }
            }
            else
            {
              // We can't infer any information regarding lanes, other than the
              // next desired waypoint.
              _tracking_state = TrackingState::TowardsWaypoint;
              _tracking_index = next_wp_index;
            }
          }
        }
      }
      else
      {
        // We will first check for the last visited waypoint.
        std::shared_ptr<requests::RelocalizationRequestInfo> reloc_req =
          std::dynamic_pointer_cast<requests::RelocalizationRequestInfo>(
            request);
        auto lanes = 
          _graph->lanes_from(reloc_req->request().last_visited_waypoint_index);

        double nearest_waypoint_dist = std::numeric_limits<double>::infinity();
        rmf_traffic::agv::Graph::Waypoint* nearest_waypoint = nullptr;

        double nearest_lane_dist = std::numeric_limits<double>::infinity();
        rmf_traffic::agv::Graph::Lane* nearest_lane = nullptr;

        for (std::size_t i : lanes)
        {
          const auto& lane = _graph->get_lane(i);
          
          // check if reached exit waypoint
          const auto& wp = _graph->get_waypoint(lane.exit().waypoint_index());
          const Eigen::Vector2d exit_loc = wp.get_location();
          const double loc_dist = (exit_loc - curr_loc).norm();
          if (loc_dist < _waypoint_dist_threshold &&
            loc_dist < nearest_waypoint_dist)
          {
            nearest_waypoint_dist = loc_dist;
            nearest_waypoint = &wp;
          }

          // check if within lane
          if (_is_within_lane(&lane, curr_loc))
          {
            const double lane_dist = _distance_to_lane(&lane, curr_loc);
            if (lane_dist < nearest_lane_dist &&
              lane_dist < _lane_dist_threshold)
            {
              nearest_lane_dist = lane_dist;
              nearest_lane = &lane;
            }
          }
        }

        if (nearest_waypoint)
        {
          _tracking_state = TrackingState::OnWaypoint;
          _tracking_index = nearest_waypoint->index();
        }
        else if (nearest_lane)
        {
          _tracking_state = TrackingState::OnLane;
          _tracking_index = nearest_lane->index();
        }
        else
        {
          // TODO(AA): Warn that the relocalization has caused the robot to be
          // lost.
          _tracking_state = TrackingState::Lost;
        }
      }
    }
    else if (_tracking_state == TrackingState::OnLane)
    {

    }
    else if (_tracking_state == TrackingState::TowardsWaypoint)
    {

    }
    else
    {

    }
  }

  _state = new_state;
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
