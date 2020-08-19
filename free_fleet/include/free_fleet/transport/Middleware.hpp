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

#ifndef INCLUDE__FREE_FLEET__TRANSPORT__MIDDLEWARE_HPP
#define INCLUDE__FREE_FLEET__TRANSPORT__MIDDLEWARE_HPP

#include <vector>
#include <memory>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>

namespace free_fleet {
namespace transport {

class Middleware
{
public:

  using Duration = rmf_traffic::Duration;

  /// Starts a service that replies with the navigation graph when called by
  /// individual robots. The service should remain active until the middleware
  /// instance goes out of scope.
  ///
  /// \param[in] graph
  ///   Navigation graph that will be used by the fleet manager and robots to
  ///   keep track of which lanes each robot is currently on, and which waypoint
  ///   they are currently heading towards.
  virtual void start_graph_service(
      std::shared_ptr<rmf_traffic::agv::Graph> graph) = 0;

  /// Requests for a navigation graph over the middleware from the fleet
  /// manager. This is a blocking function until the timeout is reached.
  ///
  /// \param[in] timeout
  ///   Duration before this request fails and times out, defaults to 10
  ///   seconds.
  ///
  /// \return
  ///   Shared pointer to the navigation graph to be used by both the fleet
  ///   manager and robot to keep track of which lane it is on and which
  ///   waypoint it is currently heading towards. If the timeout was reached or 
  ///   the service is not ready, a nullptr will be returned.
  virtual std::shared_ptr<rmf_traffic::agv::Graph> request_graph(
      Duration timeout = Duration(std::chrono::seconds(10))) = 0;

  /// Sends a robot state over the middleware to update the fleet manager. This
  /// will be called by the client running on each of the robots.
  ///
  /// \param[in] state
  ///   Most recent robot state message.
  virtual void send_state(const messages::RobotState& state) = 0;

  /// Reads any available robot states over the middleware. This will be called
  /// by the fleet manager to get the most recent states of each robot.
  ///
  /// \return
  ///   Vector of shared pointers to robot state messages.
  virtual std::vector<std::shared_ptr<messages::RobotState>> read_states() = 0;

  /// Sends a mode request over the middleware to be performed by a robot. This
  /// will be called by the fleet manager.
  ///
  /// \param[in] request
  ///   Mode request message detailing a desired mode for a robot.
  virtual void send_mode_request(const messages::ModeRequest& request) = 0;

  /// Read the most recent available mode request message over the middleware.
  /// This will be called by the clients running on the robots, before
  /// attempting to swtich to the desired mode.
  ///
  /// \return
  ///   Shared pointer to the newly received mode request message. If no message
  ///   was received, a nullptr will be returned.
  virtual std::shared_ptr<messages::ModeRequest> read_mode_request() = 0;

  /// Sends a navigation request over the middleware to be performed by a robot.
  /// This will be called by the fleet manager.
  ///
  /// \param[in]  request
  ///   Navigation request message detailing a new navigation route to be
  ///   performed by a robot.
  virtual void send_navigation_request(
      const messages::NavigationRequest& request) = 0;

  /// Read the most recent available navigation request message over the
  /// middleware. This will be called by the clients running on the robots,
  /// before attempting to perform the new navigation route.
  ///
  /// \return
  ///   Shared pointer to the newly received navigation request message. If no
  ///   message was received, a nullptr will be returned.
  virtual std::shared_ptr<messages::NavigationRequest>
      read_navigation_request() = 0;

  /// Virtual destructor
  ~Middleware() = default;
};

} // namespace transport
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__TRANSPORT__MIDDLEWARE_HPP
