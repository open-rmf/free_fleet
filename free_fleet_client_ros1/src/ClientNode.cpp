/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "ClientNode.hpp"
#include "ClientNodeConfig.hpp"

namespace free_fleet
{
namespace ros1
{

ClientNode::SharedPtr ClientNode::make(const ClientNodeConfig& _config)
{
  SharedPtr client_node = SharedPtr(new ClientNode(_config));

  ClientConfig client_config = _config.get_client_config();
  Client::SharedPtr client = Client::make(client_config);
  if (!client)
    return nullptr;

  client_node->start(Fields{
      std::move(client)
  });

  return client_node;
}

ClientNode::ClientNode(const ClientNodeConfig& _config) :
  client_node_config(_config)
{}

ClientNode::~ClientNode()
{}

void ClientNode::start(Fields _fields)
{
  fields = std::move(_fields);
}

} // namespace ros1
} // namespace free_fleet
