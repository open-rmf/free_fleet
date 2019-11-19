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

#include "Client.hpp"

namespace free_fleet
{

std::shared_ptr<Client> Client::make(
    const std::string& _fleet_name,
    Duration _publish_frequency)
{
  std::shared_ptr<Client> client(new Client(_fleet_name, _publish_frequency));
  return client;
}

Client::Client(
    const std::string& _fleet_name,
    Duration _publish_frequency)
: fleet_name(_fleet_name),
  publish_frequency(_publish_frequency)
{}

Client::~Client()
{}

} // namespace free_fleet
