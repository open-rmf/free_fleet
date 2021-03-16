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

#include <iostream>

#include <free_fleet_cyclonedds/StandardNames.hpp>
#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include <dds/dds.h>

#include "Publisher.hpp"
#include "Subscriber.hpp"
#include "messages/utils.hpp"
#include "messages/MiddlewareMessages.h"

namespace free_fleet {
namespace cyclonedds {

//==============================================================================
class CycloneDDSMiddleware::Implementation
{
public:

  Implementation()
  {}

  ~Implementation()
  {
    if (_started)
    {
      dds_return_t rc = dds_delete(_participant);
      if (rc != DDS_RETCODE_OK)
        DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));
    }
  }

  void _start()
  {
    _started = true;
  }

  bool _started = false;
  dds_entity_t _participant;
  Publisher<MiddlewareMessages_RobotState>::SharedPtr _state_pub;
  Subscriber<MiddlewareMessages_RobotState, 10>::SharedPtr _state_sub;
  Publisher<MiddlewareMessages_ModeRequest>::SharedPtr _mode_request_pub;
  Subscriber<MiddlewareMessages_ModeRequest>::SharedPtr _mode_request_sub;
  Publisher<MiddlewareMessages_NavigationRequest>::SharedPtr _nav_request_pub;
  Subscriber<MiddlewareMessages_NavigationRequest>::SharedPtr _nav_request_sub;
  Publisher<MiddlewareMessages_RelocalizationRequest>::SharedPtr
    _reloc_request_pub;
  Subscriber<MiddlewareMessages_RelocalizationRequest>::SharedPtr
    _reloc_request_sub;
};

//==============================================================================
std::shared_ptr<CycloneDDSMiddleware> CycloneDDSMiddleware::make_client(
  int dds_domain,
  const std::string& fleet_name,
  const std::string& xml_config)
{
  std::shared_ptr<CycloneDDSMiddleware> middleware(new CycloneDDSMiddleware());
  
  if(xml_config != "")
  {
    dds_entity_t err_code = dds_create_domain(dds_domain, xml_config.c_str());
    if (err_code < 0)
    {
      DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-err_code));
      return nullptr;
    }
  }

  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  if (participant <  0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  auto mode_request_sub =
    free_fleet::cyclonedds::Subscriber<MiddlewareMessages_ModeRequest>::make(
      participant,
      &MiddlewareMessages_ModeRequest_desc,
      Prefix + fleet_name + "/" + ModeRequestTopicName);
  auto nav_request_sub =
    free_fleet::cyclonedds::Subscriber<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      Prefix + fleet_name + "/" + NavigationRequestTopicName);
  auto reloc_request_sub =
    free_fleet::cyclonedds::Subscriber<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      Prefix + fleet_name + "/" + RelocalizationRequestTopicName);
  auto state_pub =
    free_fleet::cyclonedds::Publisher<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      Prefix + fleet_name + "/" + StateTopicName);

  if (!middleware || !mode_request_sub || !nav_request_sub || !reloc_request_sub
    || !state_pub)
  {
    std::cerr << "[ERROR]: Failed to create a client middleware.\n";
    return nullptr;
  }

  middleware->_pimpl->_participant = std::move(participant);
  middleware->_pimpl->_mode_request_sub = std::move(mode_request_sub);
  middleware->_pimpl->_nav_request_sub = std::move(nav_request_sub);
  middleware->_pimpl->_reloc_request_sub = std::move(reloc_request_sub);
  middleware->_pimpl->_state_pub = std::move(state_pub);
  middleware->_pimpl->_start();
  return middleware;
}

//==============================================================================
std::shared_ptr<CycloneDDSMiddleware> CycloneDDSMiddleware::make_server(
  int dds_domain,
  const std::string& fleet_name,
  const std::string& xml_config)
{
  std::shared_ptr<CycloneDDSMiddleware> middleware(new CycloneDDSMiddleware());

  if(xml_config != "")
  { 
    dds_entity_t err_code = dds_create_domain(dds_domain, xml_config.c_str());
    if (err_code < 0)
    {
      DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-err_code));
      return nullptr;
    }
  }

  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  if (participant <  0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  auto mode_request_pub =
    free_fleet::cyclonedds::Publisher<MiddlewareMessages_ModeRequest>::make(
      participant,
      &MiddlewareMessages_ModeRequest_desc,
      Prefix + fleet_name + "/" + ModeRequestTopicName);
  auto nav_request_pub =
    free_fleet::cyclonedds::Publisher<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      Prefix + fleet_name + "/" + NavigationRequestTopicName);
  auto reloc_request_pub =
    free_fleet::cyclonedds::Publisher<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      Prefix + fleet_name + "/" + RelocalizationRequestTopicName);
  auto state_sub =
    Subscriber<MiddlewareMessages_RobotState, 10>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      Prefix + fleet_name + "/" + StateTopicName);

  if (!middleware || !mode_request_pub || !nav_request_pub || !reloc_request_pub
    || !state_sub)
  {
    std::cerr << "[ERROR]: Failed to create a server middleware.\n";
    return nullptr;
  }

  middleware->_pimpl->_participant = std::move(participant);
  middleware->_pimpl->_mode_request_pub = std::move(mode_request_pub);
  middleware->_pimpl->_nav_request_pub = std::move(nav_request_pub);
  middleware->_pimpl->_reloc_request_pub = std::move(reloc_request_pub);
  middleware->_pimpl->_state_sub = std::move(state_sub);
  return middleware;
}

//==============================================================================
CycloneDDSMiddleware::~CycloneDDSMiddleware()
{}

//==============================================================================
void CycloneDDSMiddleware::send_state(const messages::RobotState& state)
{
  MiddlewareMessages_RobotState* msg = MiddlewareMessages_RobotState__alloc();
  convert(state, *msg);
  if (!_pimpl->_state_pub->write(msg))
  {
    std::cerr << "[ERROR]: Failed to publish state.\n";
  }
  MiddlewareMessages_RobotState_free(msg, DDS_FREE_ALL);
}

//==============================================================================
std::vector<messages::RobotState> CycloneDDSMiddleware::read_states()
{
  if (!_pimpl->_state_sub)
  {
    std::cerr << "Woah, subscriber died suddenly\n";
    return {};
  }

  auto msgs = _pimpl->_state_sub->read();
  if (!msgs.empty())
  {
    std::vector<messages::RobotState> states;
    states.resize(msgs.size());
    for (std::size_t i = 0; i < msgs.size(); ++i)
    {
      convert(*(msgs[i]), states[i]);
    }
    return states;
  }
  return {};
}

//==============================================================================
void CycloneDDSMiddleware::send_mode_request(
  const messages::ModeRequest& request)
{
  MiddlewareMessages_ModeRequest* msg = MiddlewareMessages_ModeRequest__alloc();
  convert(request, *msg);
  if (!_pimpl->_mode_request_pub->write(msg))
  {
    std::cerr << "[ERROR]: Failed to publish mode request.\n";
  }
  MiddlewareMessages_ModeRequest_free(msg, DDS_FREE_ALL);
}

//==============================================================================
auto CycloneDDSMiddleware::read_mode_request() -> 
  rmf_utils::optional<messages::ModeRequest>
{
  auto msgs = _pimpl->_mode_request_sub->read();
  if (!msgs.empty())
  {
    messages::ModeRequest request;
    convert(*(msgs[0]), request);
    return request;
  }
  return rmf_utils::nullopt;
}

//==============================================================================
void CycloneDDSMiddleware::send_navigation_request(
  const messages::NavigationRequest& request)
{
  MiddlewareMessages_NavigationRequest* msg =
    MiddlewareMessages_NavigationRequest__alloc();
  convert(request, *msg);
  if (!_pimpl->_nav_request_pub->write(msg))
  {
    std::cerr << "[ERROR]: Failed to publish navigation request.\n";
  }
  MiddlewareMessages_NavigationRequest_free(msg, DDS_FREE_ALL);
}

//==============================================================================
auto CycloneDDSMiddleware::read_navigation_request() ->
  rmf_utils::optional<messages::NavigationRequest>
{
  auto msgs = _pimpl->_nav_request_sub->read();
  if (!msgs.empty())
  {
    messages::NavigationRequest request;
    convert(*(msgs[0]), request);
    return request;
  }
  return rmf_utils::nullopt;
}

//==============================================================================
void CycloneDDSMiddleware::send_relocalization_request(
  const messages::RelocalizationRequest& request)
{
  MiddlewareMessages_RelocalizationRequest* msg =
    MiddlewareMessages_RelocalizationRequest__alloc();
  convert(request, *msg);
  if (!_pimpl->_reloc_request_pub->write(msg))
  {
    std::cerr << "[ERROR]: Failed to publish relocalization request.\n";
  }
  MiddlewareMessages_RelocalizationRequest_free(msg, DDS_FREE_ALL);
}

//==============================================================================
auto CycloneDDSMiddleware::read_relocalization_request() ->
  rmf_utils::optional<messages::RelocalizationRequest>
{
  auto msgs = _pimpl->_reloc_request_sub->read();
  if (!msgs.empty())
  {
    messages::RelocalizationRequest request;
    convert(*(msgs[0]), request);
    return request;
  }
  return rmf_utils::nullopt;
}

//==============================================================================
CycloneDDSMiddleware::CycloneDDSMiddleware()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet
