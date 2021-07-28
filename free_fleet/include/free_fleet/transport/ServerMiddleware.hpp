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
#include <optional>
#include <functional>

#include <rmf_traffic/Time.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
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

  /// Sets the callback function to be called whenever a robot state message is
  /// received through the middleware.
  ///
  /// \param[in] callback
  ///   Callback function for handling a new incoming robot state.
  virtual void set_robot_state_callback(
    std::function<void(const messages::RobotState&)> callback) = 0;

  /// Sends a dock request over the middleware to be performed by a robot.
  ///
  /// \param[in] request
  ///   Dock request message for a robot.
  virtual void send_dock_request(const messages::DockRequest& request) = 0;

  /// Sends a pause request over the middleware to be performed by a robot.
  ///
  /// \param[in] request
  ///   Pause request message for a robot.
  virtual void send_pause_request(const messages::PauseRequest& request) = 0;

  /// Sends a resume request over the middleware to be performed by a robot.
  ///
  /// \param[in] request
  ///   Resume request message for a robot.
  virtual void send_resume_request(const messages::ResumeRequest& request) = 0;

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
