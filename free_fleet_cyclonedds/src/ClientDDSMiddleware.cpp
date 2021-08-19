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
#include "messages/convert.hpp"

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
    if (participant > 0)
    {
      dds_return_t rc = dds_delete(participant);
      if (rc != DDS_RETCODE_OK)
        ffwarn << "dds_delete: " << dds_strretcode(-rc) << "\n";
    }
  }

  dds_entity_t participant;

  Publisher<MiddlewareMessages_RobotState>::SharedPtr state_pub;
 
  Subscriber<MiddlewareMessages_DockRequest>::SharedPtr dock_req_sub;
 
  Subscriber<MiddlewareMessages_PauseRequest>::SharedPtr pause_req_sub;
 
  Subscriber<MiddlewareMessages_ResumeRequest>::SharedPtr resume_req_sub;
 
  Subscriber<MiddlewareMessages_NavigationRequest>::SharedPtr
    navigation_req_sub;
 
  Subscriber<MiddlewareMessages_RelocalizationRequest>::SharedPtr
    relocalization_req_sub;
};

//==============================================================================
ClientDDSMiddleware::ClientDDSMiddleware()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
auto ClientDDSMiddleware::make(int dds_domain, const std::string& fleet_name)
  -> std::shared_ptr<ClientDDSMiddleware>
{
  std::shared_ptr<ClientDDSMiddleware> middleware(new ClientDDSMiddleware());
  
  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  if (participant <  0)
  {
    fferr << "dds_create_participant: " << dds_strretcode(-participant) << "\n";
    return nullptr;
  }

  auto report_fail = [p = participant](const std::string& component)
  {
    fferr << "Failed to create " << component << "\n";
    if (p > 0)
    {
      dds_return_t rc = dds_delete(p);
      if (rc != DDS_RETCODE_OK)
        ffwarn << "dds_delete: " << dds_strretcode(-rc) << "\n";
    }
    return nullptr;
  };
  
  auto state_pub =
    Publisher<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      namespacify(Prefix, fleet_name, StateTopicName));
  if (!state_pub)
    return report_fail("RobotState publisher");

  auto dock_req_sub =
    Subscriber<MiddlewareMessages_DockRequest>::make(
      participant,
      &MiddlewareMessages_DockRequest_desc,
      namespacify(Prefix, fleet_name, DockRequestTopicName));
  if (!dock_req_sub)
    return report_fail("DockRequest subscriber");

  auto pause_req_sub =
    Subscriber<MiddlewareMessages_PauseRequest>::make(
      participant,
      &MiddlewareMessages_PauseRequest_desc,
      namespacify(Prefix, fleet_name, PauseRequestTopicName));
  if (!pause_req_sub)
    return report_fail("PauseRequest subscriber");

  auto resume_req_sub =
    Subscriber<MiddlewareMessages_ResumeRequest>::make(
      participant,
      &MiddlewareMessages_ResumeRequest_desc,
      namespacify(Prefix, fleet_name, ResumeRequestTopicName));
  if (!resume_req_sub)
    return report_fail("ResumeRequest subscriber");

  auto navigation_req_sub =
    Subscriber<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      namespacify(Prefix, fleet_name, NavigationRequestTopicName));
  if (!navigation_req_sub)
    return report_fail("NavigationRequest subscriber");    

  auto relocalization_req_sub =
    Subscriber<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      namespacify(Prefix, fleet_name, RelocalizationRequestTopicName));
  if (!relocalization_req_sub)
    return report_fail("RelocalizationRequest subscriber");

  middleware->_pimpl->participant = std::move(participant);
  middleware->_pimpl->state_pub = std::move(state_pub);
  middleware->_pimpl->dock_req_sub = std::move(dock_req_sub);
  middleware->_pimpl->pause_req_sub = std::move(pause_req_sub);
  middleware->_pimpl->resume_req_sub = std::move(resume_req_sub);
  middleware->_pimpl->navigation_req_sub = std::move(navigation_req_sub);
  middleware->_pimpl->relocalization_req_sub =
    std::move(relocalization_req_sub);
  return middleware;
}

//==============================================================================
void ClientDDSMiddleware::send_state(const messages::RobotState& state)
{
  auto dds_msg = convert(state);
  if (!dds_msg.has_value())
    return;

  if (!_pimpl->state_pub->write(&dds_msg.value()))
  {
    fferr << "Failed to publish RobotState.\n";
  }
}

//==============================================================================
void ClientDDSMiddleware::set_dock_request_callback(
  std::function<void(const messages::DockRequest&)> callback)
{
  auto dds_cb =
    [c = std::move(callback)](const MiddlewareMessages_DockRequest& dds_msg)
  {
    auto converted_msg = convert(dds_msg);
    if (!converted_msg.has_value())
      return;

    c(converted_msg.value());
  };
  _pimpl->dock_req_sub->set_callback(std::move(dds_cb));
}

//==============================================================================
void ClientDDSMiddleware::set_pause_request_callback(
  std::function<void(const messages::PauseRequest&)> callback)
{
  auto dds_cb =
    [c = std::move(callback)](const MiddlewareMessages_PauseRequest& dds_msg)
  {
    auto converted_msg = convert(dds_msg);
    if (!converted_msg.has_value())
      return;

    c(converted_msg.value());
  };
  _pimpl->pause_req_sub->set_callback(std::move(dds_cb));
}

//==============================================================================
void ClientDDSMiddleware::set_resume_request_callback(
  std::function<void(const messages::ResumeRequest&)> callback)
{
  auto dds_cb =
    [c = std::move(callback)](const MiddlewareMessages_ResumeRequest& dds_msg)
  {
    auto converted_msg = convert(dds_msg);
    if (!converted_msg.has_value())
      return;

    c(converted_msg.value());
  };
  _pimpl->resume_req_sub->set_callback(std::move(dds_cb));
}

//==============================================================================
void ClientDDSMiddleware::set_navigation_request_callback(
  std::function<void(const messages::NavigationRequest&)> callback)
{
  auto dds_cb =
    [c = std::move(callback)]
    (const MiddlewareMessages_NavigationRequest& dds_msg)
  {
    auto converted_msg = convert(dds_msg);
    if (!converted_msg.has_value())
      return;

    c(converted_msg.value());
  };
  _pimpl->navigation_req_sub->set_callback(std::move(dds_cb));
}

//==============================================================================
void ClientDDSMiddleware::set_relocalization_request_callback(
  std::function<void(const messages::RelocalizationRequest&)> callback)
{
  auto dds_cb =
    [c = std::move(callback)]
    (const MiddlewareMessages_RelocalizationRequest& dds_msg)
  {
    auto converted_msg = convert(dds_msg);
    if (!converted_msg.has_value())
      return;

    c(converted_msg.value());
  };
  _pimpl->relocalization_req_sub->set_callback(std::move(dds_cb));
}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet
