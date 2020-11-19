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

#ifndef INCLUDE__FREE_FLEET__MANAGER_HPP
#define INCLUDE__FREE_FLEET__MANAGER_HPP

#include <memory>
#include <vector>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <free_fleet/transport/Middleware.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {

class Manager
{
public:

  using SharedPtr = std::shared_ptr<Manager>;

  using TimeNow = std::function<rmf_traffic::Time()>;
  using NewRobotStateCallback =
    std::function<void(const messages::RobotState& state)>;

  /// Factory function that creates an instance of the Free Fleet Manager.
  ///
  /// \param[in] fleet_name
  /// \param[in] graph
  /// \param[in] middleware
  /// \return
  static SharedPtr make(
    const std::string& fleet_name,
    std::shared_ptr<rmf_traffic::agv::Graph> graph,
    std::shared_ptr<transport::Middleware> middleware,
    TimeNow time_now_fn,
    NewRobotStateCallback new_robot_state_callback_fn);

  /// Starts the manager which begins to listen for clients.
  ///
  /// \param[in] frequency
  void start(uint32_t frequency);

  /// Gets all the names of the robots that are currently under this manager.
  ///
  /// \return
  std::vector<std::string> robots();

  /// Gets the state of the robot with the provided name. If no such robot
  /// exists, a nullopt will be returned.
  ///
  /// \param[in] robot_name
  /// \return
  rmf_utils::optional<messages::RobotState> robot_state(
    const std::string& robot_name);

  /// Gets all the most up-to-date robot states received by the manager.
  ///
  /// \return
  std::vector<messages::RobotState> robot_states();

  /// Sends out a mode request to a robot. The name of the robot can be
  /// retrieved from the contents of the message.
  ///
  /// \param[in] request
  void send_mode_request(const messages::ModeRequest& request);

  /// Sends out a navigation request to a robot. The name of the robot can be
  /// retrieved from the contents of the message.
  ///
  /// \param[in] request
  void send_navigation_request(const messages::NavigationRequest& request);

  /// Sends out a relocalization request to a robot. The name of the robot can
  /// be retrieved from the contents of the message.
  ///
  /// \param[in] request
  void send_relocalization_request(
    const messages::RelocalizationRequest& request);

  class Implementation;
private:
  Manager();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MANAGER_HPP
