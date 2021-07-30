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
#include <free_fleet_cyclonedds/ServerDDSMiddleware.hpp>


#include "Publisher.hpp"
#include "Subscriber.hpp"

namespace free_fleet {
namespace cyclonedds {

//==============================================================================
class ServerDDSMiddleware::Implementation
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

  dds_entity_t participant;

  Subscriber<MiddlewareMessages_RobotState>::SharedPtr state_sub;

  Publisher<MiddlewareMessages_DockRequest>::SharedPtr dock_req_pub;
 
  Publisher<MiddlewareMessages_PauseRequest>::SharedPtr pause_req_pub;
 
  Publisher<MiddlewareMessages_ResumeRequest>::SharedPtr resume_req_pub;
 
  Publisher<MiddlewareMessages_NavigationRequest>::SharedPtr nav_req_pub;
 
  Publisher<MiddlewareMessages_RelocalizationRequest>::SharedPtr reloc_req_pub;
};

//==============================================================================
ServerDDSMiddleware::ServerDDSMiddleware()
: _pimpl(rmf_utils::maddke_impl<Implementation>(Implementation())
{}

//==============================================================================
auto ServerDDSMiddleware::make(int dds_domain, const std::string& fleet_name)
  -> std::shared_ptr<ServerDDSMiddleware>
{
  std::shared_ptr<ServerDDSMiddleware> middleware(new ServerDDSMiddleware());
  
  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  if (participant <  0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  auto dock_req_pub =
    Publisher<MiddlewareMessages_DockRequest>::make(
      participant,
      &MiddlewareMessages_DockRequest_desc,
      append(Prefix, fleet_name, DockRequestTopicName));
  auto pause_req_pub =
    Publisher<MiddlewareMessages_PauseRequest>::make(
      participant,
      &MiddlewareMessages_PauseRequest_desc,
      append(Prefix, fleet_name, PauseRequestTopicName));
  auto resume_req_pub =
    Publisher<MiddlewareMessages_ResumeRequest>::make(
      participant,
      &MiddlewareMessages_ResumeRequest_desc,
      append(Prefix, fleet_name, ResumeRequestTopicName));
  auto nav_req_pub =
    Publisher<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      append(Prefix, fleet_name, NavigationRequestTopicName));
  auto reloc_req_pub =
    Publisher<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      append(Prefix, fleet_name, RelocalizationRequestTopicName));

  if (!dock_req_pub || !pause_req_pub || !resume_req_pub || !nav_req_pub ||
      !reloc_req_pub)
    return nullptr;

  middleware->_pimpl->participant = std::move(participant);
  middleware->_pimpl->dock_req_pub = std::move(dock_req_pub);
  middleware->_pimpl->pause_req_pub = std::move(pause_req_pub);
  middleware->_pimpl->resume_req_pub = std::move(resume_req_pub);
  middleware->_pimpl->nav_req_pub = std::move(nav_req_pub);
  middleware->_pimpl->reloc_req_pub = std::move(reloc_req_pub);
  return middleware;
}

//==============================================================================
void ServerDDSMiddleware::set_robot_state_callback(
  std::function<void(const messages::RobotState&)> callback)
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
void ServerDDSMiddleware::send_dock_request(const messages:DockRequest& request)
{}

//==============================================================================
void ServerDDSMiddleware::send_pause_request(
  const messages:PauseRequest& request)
{}

//==============================================================================
void ServerDDSMiddleware::send_resume_request(
  const messages::ResumeRequest& request)
{}

//==============================================================================
void ServerDDSMiddleware::send_navigation_request(
  const messages::NavigationRequest& request)
{}

//==============================================================================
void ServerDDSMiddleware::send_relocalization_request(
  const messages::RelocalizationRequest& request)
{}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet
