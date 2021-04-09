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

#ifndef INCLUDE__FREE_FLEET__TRANSPORT__CLIENTMIDDLEWARE_HPP
#define INCLUDE__FREE_FLEET__TRANSPORT__CLIENTMIDDLEWARE_HPP

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
/// the free fleet client.
class ClientMiddleware
{
public:

  using Duration = rmf_traffic::Duration;

  /// Sends a robot state over the middleware to be received by the server.
  ///
  /// \param[in] state
  ///   Most recent robot state message.
  virtual void send_state(const messages::RobotState& state) = 0;

  /// Sets the callback function to be called whenever a mode request is
  /// received through the middleware.
  ///
  /// \param[in] callback
  ///   Callback function for handling new incoming mode requests.
  virtual void set_mode_request_callback(
    std::function<void(const messages::ModeRequest&)> callback) = 0;

  /// Sets the callback function to be called whenever a navigation request is
  /// received through the middleware.
  ///
  /// \param[in] callback
  ///   Callback function for handling new incoming navigation requests.  
  virtual void set_navigation_request_callback(
    std::function<void(const messages::NavigationRequest&)> callback) = 0;

  /// Sets the callback function to be called whenever a relocalization
  /// request is received through the middleware.
  ///
  /// \param[in] callback
  ///   Callback function for handling new incoming relocalization requests.
  virtual void set_relocalization_request_callback(
    std::function<void(const messages::RelocalizationRequest&)> callback) = 0;

  /// Virtual destructor
  virtual ~ClientMiddleware() = default;
};

} // namespace transport
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__TRANSPORT__CLIENTMIDDLEWARE_HPP
