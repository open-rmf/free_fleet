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

#ifndef INCLUDE__FREE_FLEET_CYCLONEDDS__CLIENTDDSMIDDLEWARE_HPP
#define INCLUDE__FREE_FLEET_CYCLONEDDS__CLIENTDDSMIDDLEWARE_HPP

#include <memory>
#include <string>

#include <rmf_utils/impl_ptr.hpp>
#include <free_fleet/transport/ClientMiddleware.hpp>

namespace free_fleet {
namespace cyclonedds {

//==============================================================================
class ClientDDSMiddleware : public transport::ClientMiddleware
{
public:

  static std::shared_ptr<ClientDDSMiddleware> make(
    int dds_domain,
    const std::string& fleet_name);

  void send_state(const messages::RobotState& state) override;

  void set_dock_request_callback(
    std::function<void(const messages:DockRequest&)> callback) override;

  void set_pause_request_callback(
    std::function<void(const messages::PauseRequest&)> callback) override;

  void set_resume_request_callback(
    std::function<void(const messages::ResumeRequest&)> callback) override;

  void set_navigation_request_callback(
    std::function<void(const messages::NavigationRequest&) callback) override;

  void set_relocalization_request_callback(
    std::function<void(const messages::RelocalizationRequest&) callback)
    override;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
  ClientDDSMiddleware();
};

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET_CYCLONEDDS__CLIENTDDSMIDDLEWARE_HPP
