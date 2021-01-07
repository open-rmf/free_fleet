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

#include "internal_RobotInfo.hpp"

namespace free_fleet {
namespace agv {

//==============================================================================
void RobotInfo::Implementation::allocate_task(
  const std::shared_ptr<requests::RequestInfo>& new_request_info)
{

}

//==============================================================================
void RobotInfo::Implementation::update_state(
  const messages::RobotState& new_state,
  rmf_traffic::Time time_now)
{
  
}

//==============================================================================
void RobotInfo::Implementation::track_and_update(
  const messages::RobotState& new_state)
{

}

//==============================================================================
void RobotInfo::Implementation::track_without_task_id(
  const Eigen::Vector2d& new_state)
{

}

//==============================================================================
double RobotInfo::Implementation::distance_to_lane(
  rmf_traffic::agv::Graph::Lane* lane,
  const Eigen::Vector2d& coordinates) const
{
  return 0.0;
}

//=============================================================================
auto RobotInfo::Implementation::find_nearest_waypoint(
  const Eigen::Vector2d& coordinates) const
  -> std::pair<rmf_traffic::agv::Graph::Waypoint*, double>
{
  return std::make_pair(nullptr, 0.0);
}

//==============================================================================
auto RobotInfo::Implementation::find_nearest_lane(
  const Eigen::Vector2d& coordinates) const
  -> std::pair<rmf_traffic::agv::Graph::Lane*, double>
{
  return std::make_pair(nullptr, 0.0);
}

//==============================================================================
bool RobotInfo::Implementation::is_within_lane(
  rmf_traffic::agv::Graph::Lane* lane,
  const Eigen::Vector2d& coordinates) const
{
  return false;
}

//==============================================================================
bool RobotInfo::Implementation::is_near_waypoint(
  std::size_t waypoint_index,
  const Eigen::Vector2d& coordinates) const
{
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
std::shared_ptr<rmf_traffic::agv::Graph> RobotInfo::graph() const
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
