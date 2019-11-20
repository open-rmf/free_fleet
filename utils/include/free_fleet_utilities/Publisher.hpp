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

#ifndef INCLUDE__FREE_FLEET_UTILS__PUBLISHER_HPP
#define INCLUDE__FREE_FLEET_UTILS__PUBLISHER_HPP

#include <iostream>
#include <memory>

#include <dds/dds.h>

namespace free_fleet
{
namespace utilities
{

struct PublisherConfig
{
  std::string topic_name;
  dds_entity_t participant;
  dds_topic_descriptor* descriptor;
  dds_qos_t* qos;
  dds_listener_t* listener;
};

class Publisher
{
public:

  Publisher(const PublisherConfig& config);

  ~Publisher();

private:

  dds_entity_t participant;

  dds_entity_t topic;

  dds_entity_t writer;

};

} // namespace utilities
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET_UTILS__PUBLISHER_HPP
