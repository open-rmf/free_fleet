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
#include "src/messages/MiddlewareMessages.h"

SCENARIO("Verify that these message type subscribers can be created")
{
  // Create a DDS participant
  dds_entity_t participant = dds_create_participant(42, NULL, NULL);
  REQUIRE(participant >= 0);

  using namespace free_fleet::cyclonedds;

  GIVEN("Location message type")
  {
    auto sub = Subscriber<MiddlewareMessages_Location>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "make_Location_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("Waypoint message type")
  {
    auto sub = Subscriber<MiddlewareMessages_Waypoint>::make(
      participant,
      &MiddlewareMessages_Waypoint_desc,
      "make_Waypoint_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ModeParameter message type")
  {
    auto sub = Subscriber<MiddlewareMessages_ModeParameter>::make(
      participant,
      &MiddlewareMessages_ModeParameter_desc,
      "make_ModeParameter_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotMode message type")
  {
    auto sub = Subscriber<MiddlewareMessages_RobotMode>::make(
      participant,
      &MiddlewareMessages_RobotMode_desc,
      "make_RobotMode_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ModeRequest message type")
  {
    auto sub = Subscriber<MiddlewareMessages_ModeRequest>::make(
      participant,
      &MiddlewareMessages_ModeRequest_desc,
      "make_ModeRequest_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("NavigationRequest message type")
  {
    auto sub = Subscriber<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      "make_NavigationRequest_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RelocalizationRequest message type")
  {
    auto sub = Subscriber<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      "make_RelocalizationRequest_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotState message type")
  {
    auto sub = Subscriber<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      "make_RobotState_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }


  GIVEN("Location message type with depth 10")
  {
    auto sub = Subscriber<MiddlewareMessages_Location, 10>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "make_Location_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("Waypoint message type with depth 10")
  {
    auto sub = Subscriber<MiddlewareMessages_Waypoint, 10>::make(
      participant,
      &MiddlewareMessages_Waypoint_desc,
      "make_Waypoint_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ModeParameter message type with depth 10")
  {
    auto sub = Subscriber<MiddlewareMessages_ModeParameter, 10>::make(
      participant,
      &MiddlewareMessages_ModeParameter_desc,
      "make_ModeParameter_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotMode message type with depth 10")
  {
    auto sub = Subscriber<MiddlewareMessages_RobotMode, 10>::make(
      participant,
      &MiddlewareMessages_RobotMode_desc,
      "make_RobotMode_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ModeRequest message type with depth 10")
  {
    auto sub = Subscriber<MiddlewareMessages_ModeRequest, 10>::make(
      participant,
      &MiddlewareMessages_ModeRequest_desc,
      "make_ModeRequest_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("NavigationRequest message type with depth 10")
  {
    auto sub = Subscriber<MiddlewareMessages_NavigationRequest, 10>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      "make_NavigationRequest_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RelocalizationRequest message type with depth 10")
  {
    auto sub = Subscriber<MiddlewareMessages_RelocalizationRequest, 10>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      "make_RelocalizationRequest_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotState message type with depth 10")
  {
    auto sub = Subscriber<MiddlewareMessages_RobotState, 10>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      "make_RobotState_subscriber");
    REQUIRE(sub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }
}
