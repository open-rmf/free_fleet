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
#include "MiddlewareMessages.h"

SCENARIO("Verify that these message type publishers can be created")
{
  // Create a DDS participant
  dds_entity_t participant = dds_create_participant(42, NULL, NULL);
  REQUIRE(participant >= 0);

  using namespace free_fleet::cyclonedds;

  GIVEN("Location")
  {
    auto pub = Publisher<MiddlewareMessages_Location>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "make_Location_publisher");
    REQUIRE_FALSE(pub == nullptr);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("Waypoint")
  {
    auto pub = Publisher<MiddlewareMessages_Waypoint>::make(
      participant,
      &MiddlewareMessages_Waypoint_desc,
      "make_Waypoint_publisher");
    REQUIRE_FALSE(pub == nullptr);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotMode")
  {
    auto pub = Publisher<MiddlewareMessages_RobotMode>::make(
      participant,
      &MiddlewareMessages_RobotMode_desc,
      "make_RobotMode_publisher");
    REQUIRE_FALSE(pub == nullptr);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("PauseRequest")
  {
    auto pub = Publisher<MiddlewareMessages_PauseRequest>::make(
      participant,
      &MiddlewareMessages_PauseRequest_desc,
      "make_PauseRequest_publisher");
    REQUIRE_FALSE(pub == nullptr);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ResumeRequest")
  {
    auto pub = Publisher<MiddlewareMessages_ResumeRequest>::make(
      participant,
      &MiddlewareMessages_ResumeRequest_desc,
      "make_ResumeRequest_publisher");
    REQUIRE_FALSE(pub == nullptr);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("DockRequest")
  {
    auto pub = Publisher<MiddlewareMessages_DockRequest>::make(
      participant,
      &MiddlewareMessages_DockRequest_desc,
      "make_DockRequest_publisher");
    REQUIRE_FALSE(pub == nullptr);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("NavigationRequest message type")
  {
    auto pub = Publisher<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      "make_NavigationRequest_publisher");
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RelocalizationRequest message type")
  {
    auto pub = Publisher<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      "make_RelocalizationRequest_publisher");
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotState message type")
  {
    auto pub = Publisher<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      "make_RobotState_publisher");
    REQUIRE(pub);
    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }
}
