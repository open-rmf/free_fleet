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

#ifndef INCLUDE__FREE_FLEET_CYCLONEDDS__SERVERDDSMIDDLEWARE_HPP
#define INCLUDE__FREE_FLEET_CYCLONEDDS__SERVERDDSMIDDLEWARE_HPP

#include <memory>
#include <string>

#include <rmf_utils/impl_ptr.hpp>
#include <free_fleet/transport/ServerMiddleware.hpp>

namespace free_fleet {
namespace cyclonedds {

//==============================================================================
class ServerDDSMiddleware : public transport::ServerMiddlware
{
public:
  
  static std::shared_ptr<ServerDDSMiddleware> make(
    int dds_domain,
    const std::string& fleet_name);

  void set_robot_state_callback(
    std::function<void(const messages::RobotState&)> callback) override;

  void send_dock_request(const messages::DockRequest& request) override;

  void send_pause_request(const messages::PauseRequest& request) override;

  void send_resume_request(const messages::ResumeRequest& request) override;

  void send_navigation_request(const messages::NavigationRequest& request)
    override;

  void send_relocalization_request(
    const messages::RelocalizationRequest& request) override;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
  ServerDDSMiddleware();
};

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET_CYCLONEDDS__SERVERDDSMIDDLEWARE_HPP
