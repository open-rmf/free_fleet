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

#ifndef SRC__AGV__ROBOTINFO_HPP
#define SRC__AGV__ROBOTINFO_HPP

#include <string>
#include <memory>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <free_fleet/Manager.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/RobotState.hpp>

namespace free_fleet {
namespace agv {

class RobotInfo
{
public:

  using SharedPtr = std::shared_ptr<RobotInfo>;

  /// Gets the name of the robot.
  std::string name() const;

  /// Gets the model of the robot.
  std::string model() const;

  /// Gets a const reference to the most recent state.
  const messages::RobotState& state() const;

  /// The time stamp of when the last state update happened.
  rmf_traffic::Time last_updated() const;

  /// The time stamp of when the robot was first discovered.
  rmf_traffic::Time first_found() const;

  /// Update the internal robot handler with the newest state.
  ///
  /// \param[in] new_state
  ///   The most recent incoming state from the robot.
  ///
  /// \param[in] time_now
  ///   The current time stamp relative to the fleet manager.
  void update_state(
    const messages::RobotState& new_state,
    rmf_traffic::Time time_now);

private:
  friend class free_fleet::Manager;

  RobotInfo(
    const messages::RobotState& state,
    std::shared_ptr<rmf_traffic::agv::Graph> graph,
    rmf_traffic::Time time_now);

  std::string _name;
  std::string _model;

  rmf_traffic::Time _first_found;
  rmf_traffic::Time _last_updated;

  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
  rmf_utils::optional<std::size_t> _last_known_wp;

  messages::RobotState _state;
};

} // namespace agv
} // namespace free_fleet

#endif // SRC__AGV__ROBOTINFO_HPP
