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

  /// Read the most recent available mode request message over the middleware.
  ///
  /// \return
  ///   An optional of mode request message. If no request was received, a
  ///   nullopt is returned.
  virtual rmf_utils::optional<messages::ModeRequest> read_mode_request() = 0;

  /// Read the most recent available navigation request message over the
  /// middleware.
  ///
  /// \return
  ///   An optional of navigation request message. If no request was received,
  ///   a nullopt is returned.
  virtual rmf_utils::optional<messages::NavigationRequest>
    read_navigation_request() = 0;

  /// Read the most recent available relocalization request message over the
  /// middleware.
  ///
  /// \return
  ///   An optional of relocalization request message. If no request was
  ///   received, a nullopt is returned.
  virtual rmf_utils::optional<messages::RelocalizationRequest>
    read_relocalization_request() = 0;

  /// Virtual destructor
  virtual ~ClientMiddleware() = default;
};

} // namespace transport
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__TRANSPORT__CLIENTMIDDLEWARE_HPP
