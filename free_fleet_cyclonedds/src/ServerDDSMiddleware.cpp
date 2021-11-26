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
#include "messages/convert.hpp"

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
    if (participant > 0)
    {
      dds_return_t rc = dds_delete(participant);
      if (rc != DDS_RETCODE_OK)
        ffwarn << "dds_delete: " << dds_strretcode(-rc) << "\n";
    }
  }

  dds_entity_t participant;

  Subscriber<MiddlewareMessages_RobotState, 10>::SharedPtr state_sub;

  Publisher<MiddlewareMessages_DockRequest>::SharedPtr dock_req_pub;

  Publisher<MiddlewareMessages_PauseRequest>::SharedPtr pause_req_pub;

  Publisher<MiddlewareMessages_ResumeRequest>::SharedPtr resume_req_pub;

  Publisher<MiddlewareMessages_NavigationRequest>::SharedPtr
    navigation_req_pub;

  Publisher<MiddlewareMessages_RelocalizationRequest>::SharedPtr
    relocalization_req_pub;
};

//==============================================================================
ServerDDSMiddleware::ServerDDSMiddleware()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
}

//==============================================================================
auto ServerDDSMiddleware::make_unique(
  int dds_domain, const std::string& fleet_name)
-> std::unique_ptr<ServerDDSMiddleware>
{
  std::unique_ptr<ServerDDSMiddleware> middleware(new ServerDDSMiddleware());

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

  auto state_sub = Subscriber<MiddlewareMessages_RobotState, 10>::make(
    participant,
    &MiddlewareMessages_RobotState_desc,
    namespacify(Prefix, fleet_name, StateTopicName));
  if (!state_sub)
    return report_fail("RobotState subscriber");

  auto dock_req_pub = Publisher<MiddlewareMessages_DockRequest>::make(
    participant,
    &MiddlewareMessages_DockRequest_desc,
    namespacify(Prefix, fleet_name, DockRequestTopicName));
  if (!dock_req_pub)
    return report_fail("DockRequest publisher");

  auto pause_req_pub = Publisher<MiddlewareMessages_PauseRequest>::make(
    participant,
    &MiddlewareMessages_PauseRequest_desc,
    namespacify(Prefix, fleet_name, PauseRequestTopicName));
  if (!pause_req_pub)
    return report_fail("PauseRequest publisher");

  auto resume_req_pub = Publisher<MiddlewareMessages_ResumeRequest>::make(
    participant,
    &MiddlewareMessages_ResumeRequest_desc,
    namespacify(Prefix, fleet_name, ResumeRequestTopicName));
  if (!resume_req_pub)
    return report_fail("ResumeRequest publisher");

  auto navigation_req_pub =
    Publisher<MiddlewareMessages_NavigationRequest>::make(
    participant,
    &MiddlewareMessages_NavigationRequest_desc,
    namespacify(Prefix, fleet_name, NavigationRequestTopicName));
  if (!navigation_req_pub)
    return report_fail("NavigationRequest publisher");

  auto relocalization_req_pub =
    Publisher<MiddlewareMessages_RelocalizationRequest>::make(
    participant,
    &MiddlewareMessages_RelocalizationRequest_desc,
    namespacify(Prefix, fleet_name, RelocalizationRequestTopicName));
  if (!relocalization_req_pub)
    return report_fail("RelocalizationRequest publisher");

  middleware->_pimpl->participant = std::move(participant);
  middleware->_pimpl->state_sub = std::move(state_sub);
  middleware->_pimpl->dock_req_pub = std::move(dock_req_pub);
  middleware->_pimpl->pause_req_pub = std::move(pause_req_pub);
  middleware->_pimpl->resume_req_pub = std::move(resume_req_pub);
  middleware->_pimpl->navigation_req_pub = std::move(navigation_req_pub);
  middleware->_pimpl->relocalization_req_pub =
    std::move(relocalization_req_pub);
  return middleware;
}

//==============================================================================
auto ServerDDSMiddleware::make_shared(
  int dds_domain, const std::string& fleet_name)
-> std::shared_ptr<ServerDDSMiddleware>
{
  if (auto unique_server = make_unique(dds_domain, fleet_name))
  {
    return std::shared_ptr<ServerDDSMiddleware>(unique_server.release());
  }
  return nullptr;
}

//==============================================================================
void ServerDDSMiddleware::set_robot_state_callback(
  std::function<void(const messages::RobotState&)> callback)
{
  auto dds_cb =
    [c = std::move(callback)](const MiddlewareMessages_RobotState& dds_msg)
    {
      auto converted_msg = convert(dds_msg);
      if (!converted_msg.has_value())
        return;

      c(converted_msg.value());
    };
  _pimpl->state_sub->set_callback(std::move(dds_cb));
}

//==============================================================================
void ServerDDSMiddleware::send_dock_request(
  const messages::DockRequest& request)
{
  auto dds_msg = convert(request);
  if (!dds_msg.has_value())
    return;

  if (!_pimpl->dock_req_pub->write(&dds_msg.value()))
  {
    fferr << "Failed to publish DockRequest.\n";
  }
}

//==============================================================================
void ServerDDSMiddleware::send_pause_request(
  const messages::PauseRequest& request)
{
  auto dds_msg = convert(request);
  if (!dds_msg.has_value())
    return;

  if (!_pimpl->pause_req_pub->write(&dds_msg.value()))
  {
    fferr << "Failed to publish PauseRequest.\n";
  }
}

//==============================================================================
void ServerDDSMiddleware::send_resume_request(
  const messages::ResumeRequest& request)
{
  auto dds_msg = convert(request);
  if (!dds_msg.has_value())
    return;

  if (!_pimpl->resume_req_pub->write(&dds_msg.value()))
  {
    fferr << "Failed to publish ResumeRequest.\n";
  }
}

//==============================================================================
void ServerDDSMiddleware::send_navigation_request(
  const messages::NavigationRequest& request)
{
  auto dds_msg = convert(request);
  if (!dds_msg.has_value())
    return;

  if (!_pimpl->navigation_req_pub->write(&dds_msg.value()))
  {
    fferr << "Failed to publish NavigationRequest.\n";
  }
}

//==============================================================================
void ServerDDSMiddleware::send_relocalization_request(
  const messages::RelocalizationRequest& request)
{
  auto dds_msg = convert(request);
  if (!dds_msg.has_value())
    return;

  if (!_pimpl->relocalization_req_pub->write(&dds_msg.value()))
  {
    fferr << "Failed to publish RelocalizationRequest.\n";
  }
}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet
