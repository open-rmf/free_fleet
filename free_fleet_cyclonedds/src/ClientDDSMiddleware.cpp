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

#include <free_fleet/Console.hpp>
#include <free_fleet_cyclonedds/StandardNames.hpp>
#include <free_fleet_cyclonedds/ClientDDSMiddleware.hpp>

#include "Publisher.hpp"
#include "Subscriber.hpp"

namespace free_fleet {
namespace cyclonedds {

//==============================================================================
class ClientDDSMiddleware::Implementation
{
public:

  Implementation()
  {}

  ~Implementation()
  {
    dds_return_t rc = dds_delete(_participant);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));
  }

  dds_entity_t _participant;

  Publisher<MiddlwareMessages_RobotState>::SharedPtr state_pub;
 
  Subscriber<MiddlewareMessages_DockRequest>::SharedPtr dock_req_sub;
 
  Subscriber<MiddlewareMessages_PauseRequest>::SharedPtr pause_req_sub;
 
  Subscriber<MiddlewareMessages_ResumeRequest>::SharedPtr resume_req_sub;
 
  Subscriber<MiddlewareMessages_NavigationRequest>::SharedPtr nav_req_sub;
 
  Subscriber<MiddlewareMessages_RelocalizationRequest>::SharedPtr reloc_req_sub;
};

//==============================================================================
ClientDDSMiddleware::ClientDDSMiddleware()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation())
{}

//==============================================================================
auto ClientDDSMiddleware::make(int dds_domain, const std::string& fleet_name)
  -> std::shared_ptr<ClientDDSMiddleware>
{
  std::shared_ptr<ClientDDSMiddleware> middleware(new ClientDDSMiddleware());
  
  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  if (participant <  0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  auto state_pub =
    free_fleet::cyclonedds::Publisher<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      append(Prefix, fleet_name, StateTopicName));
  if (!state_pub)
    return nullptr;

  middleware->_pimpl->_participant = std::move(participant);
  middleware->_pimpl->_state_pub = std::move(state_pub);
  return middleware;
}

//==============================================================================
void ClientDDSMiddleware::send_state(const messages::RobotState& state)
{
  // MiddlewareMessages_RobotState* msg = MiddlewareMessages_RobotState__alloc();  
  // convert(state, *msg);                                                         
  // if (!_pimpl->_state_pub->write(msg))                                          
  // {                                                                             
  //   std::cerr << "[ERROR]: Failed to publish state.\n";                         
  // }                                                                             
  // MiddlewareMessages_RobotState_free(msg, DDS_FREE_ALL);
}

//==============================================================================
void ClientDDSMiddleware::set_dock_request_callback(
  std::function<void(const messages:DockRequest&)> callback)
{}

//==============================================================================
void ClientDDSMiddleware::set_pause_request_callback(
  std::function<void(const messages::PauseRequest&)> callback)
{}

//==============================================================================
void ClientDDSMiddleware::set_resume_request_callback(
  std::function<void(const messages::ResumeRequest&)> callback)
{}

//==============================================================================
void ClientDDSMiddleware::set_navigation_request_callback(
  std::function<void(const messages::NavigationRequest&)> callback)
{}

//==============================================================================
void ClientDDSMiddleware::set_relocalization_request_callback(
  std::function<void(const messages::RelocalizationRequest&)> callback)
{}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet
