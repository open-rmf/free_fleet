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

#ifndef SRC__MANAGER__INTERNAL__ROBOTINFO_HPP
#define SRC__MANAGER__INTERNAL__ROBOTINFO_HPP

#include <iostream>

#include <rmf_utils/optional.hpp>

#include <free_fleet/Console.hpp>
#include <free_fleet/manager/RobotInfo.hpp>

#include "../manager/requests/RequestInfo.hpp"

namespace free_fleet {
namespace manager {

class RobotInfo::Implementation
{
public:

  std::string name;
  std::string model;
  rmf_traffic::Time first_found;
  rmf_traffic::Time last_updated;
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  std::size_t tracking_index;
  TrackingState tracking_state = TrackingState::Lost;
  std::optional<messages::RobotState> state = std::nullopt;

  std::unordered_map<uint32_t, std::shared_ptr<RequestInfo>>
    allocated_requests;

  const double waypoint_dist_threshold = 0.5;
  const double lane_dist_threshold = 1.0;

  static std::shared_ptr<RobotInfo> make(
    const messages::RobotState& state,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    rmf_traffic::Time time_now)
  {
    auto make_error_fn = [](const std::string& error_msg)
    {
      fferr << error_msg << "\n";
      return nullptr;
    };

    if (state.name.empty())
      return make_error_fn("Provided robot name in state must not be empty.");
    if (state.model.empty())
      return make_error_fn("Provided robot model in state must not be empty.");
    if (!graph)
      return make_error_fn("Provided traffic graph is invalid.");

    std::shared_ptr<RobotInfo> info(new RobotInfo);
    info->_pimpl = rmf_utils::make_impl<Implementation>(Implementation());
    info->_pimpl->name = state.name;
    info->_pimpl->model = state.model;
    info->_pimpl->first_found = time_now;
    info->_pimpl->last_updated = time_now;
    info->_pimpl->graph = std::move(graph);
    update_state(*info, state, time_now);
    return info;
  }

  /// Static getter for a mutable Implementation of the provided RobotInfo.
  static Implementation& get(RobotInfo& robot_info)
  {
    return *robot_info._pimpl;
  }

  /// Static getter for a const Implementation of the provided RobotInfo.
  static const Implementation& get(const RobotInfo& robot_info)
  {
    return *robot_info._pimpl;
  }

  /// Static state updater. Attempts to update and track the robot using the new
  /// incoming state and prior information.
  static void update_state(
    RobotInfo& robot_info,
    const messages::RobotState& new_state,
    rmf_traffic::Time time_now);

  /// Allocates this task to this robot.
  ///
  /// \param[in] new_request_info
  ///   Pointer to a request.
  void allocate_task(
    const std::shared_ptr<RequestInfo>& new_request_info);

  /// Gets the tracking estimation purely based on the incoming new state as
  /// well as it's previous tracking estimation.
  std::pair<TrackingState, std::size_t> track_through_graph(
    const messages::RobotState& new_state) const;
};

} // namespace manager
} // namespace free_fleet

#endif // SRC__MANAGER__INTERNAL__ROBOTINFO_HPP
