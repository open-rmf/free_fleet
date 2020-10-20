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
  Publisher<MiddlewareMessages_Graph>::SharedPtr _graph_pub;
  Subscriber<MiddlewareMessages_Graph, 1>::SharedPtr _graph_sub;
  Publisher<MiddlewareMessages_RobotState>::SharedPtr _state_pub;
  Subscriber<MiddlewareMessages_RobotState, 10>::SharedPtr _state_sub;
  Publisher<MiddlewareMessages_ModeRequest>::SharedPtr _mode_request_pub;
  Subscriber<MiddlewareMessages_ModeRequest>::SharedPtr _mode_request_sub;
  Publisher<MiddlewareMessages_NavigationRequest>::SharedPtr _nav_request_pub;
  Subscriber<MiddlewareMessages_NavigationRequest>::SharedPtr _nav_request_sub;
};

//==============================================================================
std::shared_ptr<CycloneDDSMiddleware> CycloneDDSMiddleware::make_client(
    int dds_domain,
    const std::string& fleet_name)
{
  std::shared_ptr<CycloneDDSMiddleware> middleware(new CycloneDDSMiddleware());
  
  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  if (participant <  0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  auto graph_sub = 
    free_fleet::cyclonedds::Subscriber<MiddlewareMessages_Graph>::make(
      participant,
      &MiddlewareMessages_Graph_desc,
      Prefix + fleet_name + "/" + GraphTopicName);
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
  auto state_pub =
    free_fleet::cyclonedds::Publisher<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      Prefix + fleet_name + "/" + StateTopicName);

  if (!middleware || !graph_sub || !mode_request_sub || !nav_request_sub
    || !state_pub)
  {
    std::cerr << "[ERROR]: Failed to create a client middleware.\n";
    return nullptr;
  }

  middleware->_pimpl->_participant = std::move(participant);
  middleware->_pimpl->_graph_sub = std::move(graph_sub);
  middleware->_pimpl->_mode_request_sub = std::move(mode_request_sub);
  middleware->_pimpl->_nav_request_sub = std::move(nav_request_sub);
  middleware->_pimpl->_state_pub = std::move(state_pub);
  middleware->_pimpl->_start();
  return middleware;
}

//==============================================================================
std::shared_ptr<CycloneDDSMiddleware> CycloneDDSMiddleware::make_manager(
    int dds_domain,
    const std::string& fleet_name)
{
  std::shared_ptr<CycloneDDSMiddleware> middleware(new CycloneDDSMiddleware());

  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  if (participant <  0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  auto graph_pub =
    free_fleet::cyclonedds::Publisher<MiddlewareMessages_Graph>::make(
      participant,
      &MiddlewareMessages_Graph_desc,
      Prefix + fleet_name + "/" + GraphTopicName);
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
  auto state_sub =
    free_fleet::cyclonedds::Subscriber<MiddlewareMessages_RobotState, 10>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      Prefix + fleet_name + "/" + StateTopicName);

  if (!middleware || !graph_pub || !mode_request_pub || !nav_request_pub ||
    !state_sub)
  {
    std::cerr << "[ERROR]: Failed to create a server middleware.\n";
    return nullptr;
  }

  middleware->_pimpl->_participant = std::move(participant);
  middleware->_pimpl->_graph_pub = std::move(graph_pub);
  middleware->_pimpl->_mode_request_pub = std::move(mode_request_pub);
  middleware->_pimpl->_nav_request_pub = std::move(nav_request_pub);
  middleware->_pimpl->_state_sub = std::move(state_sub);
  return middleware;
}

//==============================================================================
CycloneDDSMiddleware::~CycloneDDSMiddleware()
{}

//==============================================================================
void CycloneDDSMiddleware::send_graph(
      std::shared_ptr<rmf_traffic::agv::Graph> graph)
{
  MiddlewareMessages_Graph* msg = MiddlewareMessages_Graph__alloc();
  convert(*graph, *msg);
  if (!_pimpl->_graph_pub->write(msg))
  {
    std::cerr << "[ERROR]: Failed to publish graph.\n";
  }
  MiddlewareMessages_Graph_free(msg, DDS_FREE_ALL);
}

//==============================================================================
std::shared_ptr<rmf_traffic::agv::Graph> CycloneDDSMiddleware::read_graph()
{
  auto msgs = _pimpl->_graph_sub->read();
  if (!msgs.empty())
  {
    std::shared_ptr<rmf_traffic::agv::Graph> graph(
      new rmf_traffic::agv::Graph());
    convert(*(msgs[0]), *graph);
    return graph;
  }
  return nullptr;
}

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
std::vector<std::shared_ptr<messages::RobotState>> 
    CycloneDDSMiddleware::read_states()
{
  auto msgs = _pimpl->_state_sub->read();
  if (!msgs.empty())
  {
    std::vector<std::shared_ptr<messages::RobotState>> states;
    for (auto s : msgs)
    {
      std::shared_ptr<messages::RobotState> state(
        new messages::RobotState);
      convert(*s, *state);
      states.push_back(std::move(state));
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
std::shared_ptr<messages::ModeRequest> CycloneDDSMiddleware::read_mode_request()
{
  auto msgs = _pimpl->_mode_request_sub->read();
  if (!msgs.empty())
  {
    std::shared_ptr<messages::ModeRequest> request(
      new messages::ModeRequest);
    convert(*(msgs[0]), *request);
    return request;
  }
  return nullptr;
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
std::shared_ptr<messages::NavigationRequest> 
    CycloneDDSMiddleware::read_navigation_request()
{
  auto msgs = _pimpl->_nav_request_sub->read();
  if (!msgs.empty())
  {
    std::shared_ptr<messages::NavigationRequest> request(
      new messages::NavigationRequest);
    convert(*(msgs[0]), *request);
    return request;
  }
  return nullptr;
}

//==============================================================================
CycloneDDSMiddleware::CycloneDDSMiddleware()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet
