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

#include <chrono>
#include <thread>
#include <iostream>

#include <dds/dds.h>
#include <rmf_utils/catch.hpp>

#include "src/Publisher.hpp"
#include "src/Subscriber.hpp"
#include "MiddlewareMessages.h"
#include "src/messages/convert.hpp"

SCENARIO("Publishing with Subscriber")
{
  dds_entity_t participant = dds_create_participant(42, NULL, NULL);
  REQUIRE(participant >= 0);

  using namespace free_fleet::cyclonedds;

  auto pub = Publisher<MiddlewareMessages_Location>::make(
    participant,
    &MiddlewareMessages_Location_desc,
    "Location_pubsub");
  REQUIRE_FALSE(pub == nullptr);

  MiddlewareMessages_Location* msg = MiddlewareMessages_Location__alloc();
  std::string map_name = "test_basic_pub_sub";
  msg->map_name = dds_string_alloc_and_copy(map_name);
  REQUIRE_FALSE(msg->map_name == nullptr);
  CHECK(std::string(msg->map_name) == "test_basic_pub_sub");

  std::string map_name_received = "";
  auto cb = [&](const MiddlewareMessages_Location& msg)
    {
      map_name_received = std::string(msg.map_name);
    };
  auto sub = Subscriber<MiddlewareMessages_Location, 10>::make(
    participant,
    &MiddlewareMessages_Location_desc,
    "Location_pubsub",
    cb);
  REQUIRE_FALSE(sub == nullptr);

  REQUIRE(pub->write(msg));
  dds_sleepfor(DDS_MSECS(50));

  CHECK(map_name_received == map_name);

  // Test with newly passed in callback
  auto new_cb = [&](const MiddlewareMessages_Location& msg)
    {
      map_name_received = std::string(msg.map_name) + "_new";
    };
  sub->set_callback(new_cb);

  REQUIRE(pub->write(msg));
  dds_sleepfor(DDS_MSECS(50));

  CHECK(map_name_received == map_name + "_new");

  MiddlewareMessages_Location_free(msg, DDS_FREE_ALL);
  dds_return_t rc = dds_delete(participant);
  REQUIRE(rc == DDS_RETCODE_OK);
}
