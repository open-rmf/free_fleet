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

#ifndef TEST__UNIT__MOCK_SERVERMIDDLEWARE_HPP
#define TEST__UNIT__MOCK_SERVERMIDDLEWARE_HPP

#include <free_fleet/transport/ServerMiddleware.hpp>

namespace free_fleet {

class MockServerMiddleware : public transport::ServerMiddleware
{
public:
  std::optional<messages::DockRequest> _prev_dock_request;
  std::optional<messages::PauseRequest> _prev_pause_request;
  std::optional<messages::ResumeRequest> _prev_resume_request;
  std::optional<messages::NavigationRequest> _prev_nav_request;
  std::optional<messages::RelocalizationRequest> _prev_reloc_request;
  std::function<void(const messages::RobotState&)> robot_state_callback; 

  MockServerMiddleware()
  {}

  void set_robot_state_callback(
    std::function<void(const messages::RobotState&)> callback) override
  {
    robot_state_callback = std::move(callback);
  }

  void send_dock_request(const messages::DockRequest& request) override
  {
    _prev_dock_request = request;
  }

  void send_pause_request(const messages::PauseRequest& request) override
  {
    _prev_pause_request = request;
  }

  void send_resume_request(const messages::ResumeRequest& request) override
  {
    _prev_resume_request = request;
  }

  void send_navigation_request(const messages::NavigationRequest& request)
    override
  {
    _prev_nav_request = request;
  }

  void send_relocalization_request(
    const messages::RelocalizationRequest& request) override
  {
    _prev_reloc_request = request;
  }
};

} // namespace free_fleet

#endif // TEST__UNIT__MOCK_SERVERMIDDLEWARE_HPP
