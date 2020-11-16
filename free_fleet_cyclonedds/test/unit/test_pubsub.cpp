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

#include <dds/dds.h>
#include <rmf_utils/catch.hpp>

#include "src/Publisher.hpp"
#include "src/Subscriber.hpp"
#include "src/messages/utils.hpp"
#include "src/messages/MiddlewareMessages.h"

SCENARIO("Verify that publishing and subscribing works")
{
  // Create a DDS participant
  dds_entity_t participant = dds_create_participant(42, NULL, NULL);
  REQUIRE(participant >= 0);

  using namespace free_fleet::cyclonedds;

  GIVEN("Basic publisher and subscriber")
  {
    auto pub = Publisher<MiddlewareMessages_Location>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "Location_pubsub");
    REQUIRE(pub);
    auto sub = Subscriber<MiddlewareMessages_Location, 10>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "Location_pubsub");
    REQUIRE(sub);

    MiddlewareMessages_Location* msg = MiddlewareMessages_Location__alloc();
    REQUIRE(pub->write(msg));

    bool received = false;
    for (int i = 0; i < 5; ++i)
    {
      auto messages = sub->read();
      if (!messages.empty())
      {
        CHECK(messages[0]);
        received = true;
        break;
      }
    }
    CHECK(received);

    MiddlewareMessages_Location_free(msg, DDS_FREE_ALL);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("Transient local publisher and subscribers")
  {
    auto pub = Publisher<MiddlewareMessages_Location>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "Location_pubsub",
      true);
    REQUIRE(pub);

    MiddlewareMessages_Location* msg = MiddlewareMessages_Location__alloc();
    std::string level_name = "test_level";
    msg->level_name = dds_string_alloc_and_copy(level_name);
    REQUIRE(pub->write(msg));

    // Creating new subscribers and subscribing should yield the same message
    for (int i = 0; i < 5; ++i)
    {
      auto sub = Subscriber<MiddlewareMessages_Location, 10>::make(
        participant,
        &MiddlewareMessages_Location_desc,
        "Location_pubsub",
        true);
      REQUIRE(sub);

      bool received = false;
      for (int i = 0; i < 5; ++i)
      {
        auto messages = sub->read();
        if (!messages.empty())
        {
          CHECK(messages[0]);
          CHECK(messages[0]->level_name == level_name);
          received = true;
          break;
        }
      }
      CHECK(received);
    }
  }
}
