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

#include "SimpleRequestInfo.hpp"
#include "../internal_RobotInfo.hpp"
#include "../utilities/utilities.hpp"

#include <free_fleet/Console.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {
namespace manager {

//==============================================================================
template <>
auto SimpleRequestInfo<messages::DockRequest>::track_robot(
  const RobotInfo& robot_info,
  const messages::RobotState& new_state) const
  -> std::pair<RobotInfo::TrackingState, std::size_t>
{
  return RobotInfo::Implementation::get(robot_info).track_through_graph(
    new_state);
}

//==============================================================================
template <>
auto SimpleRequestInfo<messages::PauseRequest>::track_robot(
  const RobotInfo& robot_info,
  const messages::RobotState& new_state) const
  -> std::pair<RobotInfo::TrackingState, std::size_t>
{
  return RobotInfo::Implementation::get(robot_info).track_through_graph(
    new_state);
}

//==============================================================================
template <>
auto SimpleRequestInfo<messages::ResumeRequest>::track_robot(
  const RobotInfo& robot_info,
  const messages::RobotState& new_state) const
  -> std::pair<RobotInfo::TrackingState, std::size_t>
{
  return RobotInfo::Implementation::get(robot_info).track_through_graph(
    new_state);
}

//==============================================================================
template <>
auto SimpleRequestInfo<messages::RelocalizationRequest>::track_robot(
  const RobotInfo& robot_info,
  const messages::RobotState& new_state) const
  -> std::pair<RobotInfo::TrackingState, std::size_t>
{
  const Eigen::Vector2d curr_loc = new_state.location().coordinates();
  auto impl = RobotInfo::Implementation::get(robot_info);
  
  double distance_to_wp =
    distance_to_waypoint(
      impl.graph->get_waypoint(_request.last_visited_waypoint_index()),
      curr_loc);

  if (distance_to_wp < impl.waypoint_dist_threshold)
  {
    // We are very close to the provided waypoint
    return std::make_pair(
      RobotInfo::TrackingState::OnWaypoint,
      _request.last_visited_waypoint_index());
  }

  // We will try to localize just based on the waypoints in the navigation
  // graph, otherwise it is considered to be lost.
  return impl.track_through_graph(new_state);
}

//==============================================================================
template <>
auto SimpleRequestInfo<messages::NavigationRequest>::track_robot(
  const RobotInfo& robot_info,
  const messages::RobotState& new_state) const
  -> std::pair<RobotInfo::TrackingState, std::size_t>
{
  // We will use the target waypoints in the navigation request as
  // additional information to help with tracking.

  const Eigen::Vector2d curr_loc = new_state.location().coordinates();
  auto prev_estimation = robot_info.tracking_estimation();
  auto impl = RobotInfo::Implementation::get(robot_info);

  const std::size_t next_wp_index =
    _request.path()[new_state.target_path_index()].index();
  
  std::optional<std::size_t> prev_wp_index = std::nullopt;
  const rmf_traffic::agv::Graph::Lane* curr_lane = nullptr;
  if (new_state.target_path_index() != 0)
  {
    prev_wp_index = _request.path()[new_state.target_path_index() - 1].index();
    curr_lane = impl.graph->lane_from(prev_wp_index.value(), next_wp_index);
  }

  double distance_to_wp =
    distance_to_waypoint(impl.graph->get_waypoint(next_wp_index), curr_loc);
  if (distance_to_wp < impl.waypoint_dist_threshold)
  {
    // The robot has reached the next waypoint on this navigation request.
    return std::make_pair(
      RobotInfo::TrackingState::OnWaypoint, next_wp_index);
  }
  else if (prev_wp_index.has_value())
  {
    double distance_to_prev_wp =
      distance_to_waypoint(
        impl.graph->get_waypoint(prev_wp_index.value()), curr_loc);
    if (distance_to_prev_wp < impl.waypoint_dist_threshold)
    {
      // The robot is still in the previous waypoint on this navigation
      // request.
      return std::make_pair(
        RobotInfo::TrackingState::OnWaypoint, prev_wp_index.value());
    }
    else if (curr_lane && is_within_lane(*curr_lane, *impl.graph, curr_loc))
    {
      double dist_to_lane = distance_to_lane(*curr_lane, *impl.graph, curr_loc);
      if (dist_to_lane > impl.lane_dist_threshold)
      {
        ffwarn << "Robot [" << robot_info.name() << "] is "
          << dist_to_lane << "m away from the lane center.\n";
      }

      return std::make_pair(
        RobotInfo::TrackingState::OnLane, curr_lane->index());
    }
  }
  
  ffwarn << "Robot [" << robot_info.name() << "] is far away "
    "from the next waypoint, and there is no path from the previous "
    "waypoint, it is LOST until a trackable state comes in.\n";
  return std::make_pair(
    RobotInfo::TrackingState::Lost, prev_estimation.second);
}

//==============================================================================
} // namespace manager
} // namesapce free_fleet
