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
#include "src/messages/MiddlewareMessages.h"

SCENARIO("Verify that these message type transient local publishers can be"
  " created")
{
  // Create a DDS participant
  dds_entity_t participant = dds_create_participant(42, NULL, NULL);
  REQUIRE(participant >= 0);

  using namespace free_fleet::cyclonedds;

  GIVEN("Location message type")
  {
    auto pub = Publisher<MiddlewareMessages_Location>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "make_Location_publisher",
      true);
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("Waypoint message type")
  {
    auto pub = Publisher<MiddlewareMessages_Waypoint>::make(
      participant,
      &MiddlewareMessages_Waypoint_desc,
      "make_Waypoint_publisher",
      true);
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ModeParameter message type")
  {
    auto pub = Publisher<MiddlewareMessages_ModeParameter>::make(
      participant,
      &MiddlewareMessages_ModeParameter_desc,
      "make_ModeParameter_publisher",
      true);
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotMode message type")
  {
    auto pub = Publisher<MiddlewareMessages_RobotMode>::make(
      participant,
      &MiddlewareMessages_RobotMode_desc,
      "make_RobotMode_publisher",
      true);
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ModeRequest message type")
  {
    auto pub = Publisher<MiddlewareMessages_ModeRequest>::make(
      participant,
      &MiddlewareMessages_ModeRequest_desc,
      "make_ModeRequest_publisher",
      true);
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("NavigationRequest message type")
  {
    auto pub = Publisher<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      "make_NavigationRequest_publisher",
      true);
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RelocalizationRequest message type")
  {
    auto pub = Publisher<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      "make_RelocalizationRequest_publisher",
      true);
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotState message type")
  {
    auto pub = Publisher<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      "make_RobotState_publisher",
      true);
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

}
