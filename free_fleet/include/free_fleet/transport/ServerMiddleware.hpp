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

#ifndef INCLUDE__FREE_FLEET__TRANSPORT__SERVERMIDDLEWARE_HPP
#define INCLUDE__FREE_FLEET__TRANSPORT__SERVERMIDDLEWARE_HPP

#include <vector>
#include <memory>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/optional.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {
namespace transport {

//==============================================================================
/// Implement this class with a selected middleware for communication used by
/// the free fleet server.
class ServerMiddleware
{
public:

  using Duration = rmf_traffic::Duration;

  /// Reads any available robot states over the middleware.
  ///
  /// \return
  ///   Vector of robot state messages.
  virtual std::vector<messages::RobotState> read_states() = 0;

  /// Sends a mode request over the middleware to be performed by a robot.
  ///
  /// \param[in] request
  ///   Mode request message detailing a desired mode for a robot.
  virtual void send_mode_request(const messages::ModeRequest& request) = 0;

  /// Sends a navigation request over the middleware to be performed by a robot.
  ///
  /// \param[in]  request
  ///   Navigation request message detailing a new navigation route to be
  ///   performed by a robot.
  virtual void send_navigation_request(
    const messages::NavigationRequest& request) = 0;

  /// Sends a relocalization request over the middleware to be performed by a
  /// robot.
  ///
  /// \param[in] request
  ///   Relocalization request message detailing the current estimated location
  ///   of the robot.
  virtual void send_relocalization_request(
    const messages::RelocalizationRequest& request) = 0;

  /// Virtual destructor
  virtual ~ServerMiddleware() = default;
};

} // namespace transport
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__TRANSPORT__MIDDLEWARE_HPP
