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

#include "src/Subscriber.hpp"
#include "MiddlewareMessages.h"

SCENARIO("Verify that these message type subscribers can be created")
{
  // Create a DDS participant
  dds_entity_t participant = dds_create_participant(42, NULL, NULL);
  REQUIRE(participant >= 0);

  using namespace free_fleet::cyclonedds;

  GIVEN("Location")
  {
    auto cb = [](const MiddlewareMessages_Location&) {};
    auto sub = Subscriber<MiddlewareMessages_Location>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "make_Location_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("Waypoint")
  {
    auto cb = [](const MiddlewareMessages_Waypoint&) {};
    auto sub = Subscriber<MiddlewareMessages_Waypoint>::make(
      participant,
      &MiddlewareMessages_Waypoint_desc,
      "make_Waypoint_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotMode")
  {
    auto cb = [](const MiddlewareMessages_RobotMode&) {};
    auto sub = Subscriber<MiddlewareMessages_RobotMode>::make(
      participant,
      &MiddlewareMessages_RobotMode_desc,
      "make_RobotMode_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("PauseRequest")
  {
    auto cb = [](const MiddlewareMessages_PauseRequest&) {};
    auto sub = Subscriber<MiddlewareMessages_PauseRequest>::make(
      participant,
      &MiddlewareMessages_PauseRequest_desc,
      "make_PauseRequest_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ResumeRequest")
  {
    auto cb = [](const MiddlewareMessages_ResumeRequest&) {};
    auto sub = Subscriber<MiddlewareMessages_ResumeRequest>::make(
      participant,
      &MiddlewareMessages_ResumeRequest_desc,
      "make_ResumeRequest_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("DockRequest")
  {
    auto cb = [](const MiddlewareMessages_DockRequest&) {};
    auto sub = Subscriber<MiddlewareMessages_DockRequest>::make(
      participant,
      &MiddlewareMessages_DockRequest_desc,
      "make_DockRequest_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("NavigationRequest message type")
  {
    auto cb = [](const MiddlewareMessages_NavigationRequest&) {};
    auto sub = Subscriber<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      "make_NavigationRequest_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RelocalizationRequest")
  {
    auto cb = [](const MiddlewareMessages_RelocalizationRequest&) {};
    auto sub = Subscriber<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      "make_RelocalizationRequest_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotState")
  {
    auto cb = [](const MiddlewareMessages_RobotState&) {};
    auto sub = Subscriber<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      "make_RobotState_subscriber",
      cb);
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }
}
