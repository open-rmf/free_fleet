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

#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include "Publisher.hpp"
#include "Subscriber.hpp"
#include "messages/MiddlewareMessages.h"

namespace free_fleet {
namespace cyclonedds {

//==============================================================================

class CycloneDDSMiddleware::Implementation
{
public:

  Implementation()
  {}

  Publisher::SharedPtr<MiddlewareMessages_Graph> _graph_pub;
  Subscriber::SharedPtr<MiddlewareMessages_Graph, 1> _graph_sub;
  Publisher::SharedPtr<MiddlewareMessages_RobotState> _state_pub;
  Subscriber::SharedPtr<MiddlewareMessages_RobotState> _state_sub;
  Publisher::SharedPtr<MiddlewareMessages_ModeRequest> _mode_request_pub;
  Subscriber::SharedPtr<MiddlewareMessages_ModeRequest> _mode_request_sub;
  Publisher::SharedPtr<MiddlewareMessages_NavigationRequest> _nav_request_pub;
  Subscriber::SharedPtr<MiddlewareMessages_NavigationRequest> _nav_request_sub;

};

//==============================================================================

std::shared_ptr<CycloneDDSMiddleware> CycloneDDSMiddleware::make_client(
    Config config)
{
  std::shared_ptr<CycloneDDSMiddleware> middleware(new CycloneDDSMiddleware());
  return middleware;
}

//==============================================================================

std::shared_ptr<CycloneDDSMiddleware> CycloneDDSMiddleware::make_manager(
    Config config)
{
  std::shared_ptr<CycloneDDSMiddleware> middleware(new CycloneDDSMiddleware());
  return middleware;
}

//==============================================================================

CycloneDDSMiddleware::~CycloneDDSMiddleware()
{}

//==============================================================================

void CycloneDDSMiddleware::send_graph(
      std::shared_ptr<rmf_traffic::agv::Graph> graph)
{}

//==============================================================================

std::shared_ptr<rmf_traffic::agv::Graph> CycloneDDSMiddleware::request_graph()
{
  return nullptr;
}

//==============================================================================

void CycloneDDSMiddleware::send_state(const messages::RobotState& state)
{}

//==============================================================================

std::vector<std::shared_ptr<messages::RobotState>> 
    CycloneDDSMiddleware::read_states()
{
  return {};
}

//==============================================================================

void CycloneDDSMiddleware::send_mode_request(
    const messages::ModeRequest& request)
{}

//==============================================================================

std::shared_ptr<messages::ModeRequest> CycloneDDSMiddleware::read_mode_request()
{
  return nullptr;
}

//==============================================================================

void CycloneDDSMiddleware::send_navigation_request(
    const messages::NavigationRequest& request)
{}

//==============================================================================
  
std::shared_ptr<messages::NavigationRequest> 
    CycloneDDSMiddleware::read_navigation_request()
{}

//==============================================================================

CycloneDDSMiddleware::CycloneDDSMiddleware()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================

} // namespace cyclonedds
} // namespace free_fleet
