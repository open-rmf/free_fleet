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

#include <rmf_utils/catch.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include <free_fleet_cyclonedds/StandardNames.hpp>
#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include "src/Publisher.hpp"
#include "src/Subscriber.hpp"
#include "src/messages/utils.hpp"
#include "src/messages/MiddlewareMessages.h"

SCENARIO("Single client with mock subscribers and publishers")
{
  using namespace free_fleet::cyclonedds;
  int dds_domain = 42;
  std::string fleet_name = "test_fleet";

  auto client = CycloneDDSMiddleware::make_client(dds_domain, fleet_name);
  REQUIRE(client);

  // Create a DDS participant
  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  REQUIRE(participant >= 0);

  auto state_sub = Subscriber<MiddlewareMessages_RobotState>::make(
    participant,
    &MiddlewareMessages_RobotState_desc,
    Prefix + fleet_name + "/" + StateTopicName);
  REQUIRE(state_sub);

  auto mode_req_pub = Publisher<MiddlewareMessages_ModeRequest>::make(
    participant,
    &MiddlewareMessages_ModeRequest_desc,
    Prefix + fleet_name + "/" + ModeRequestTopicName);
  REQUIRE(mode_req_pub);

  auto nav_req_pub = Publisher<MiddlewareMessages_NavigationRequest>::make(
    participant,
    &MiddlewareMessages_NavigationRequest_desc,
    Prefix + fleet_name + "/" + NavigationRequestTopicName);
  REQUIRE(nav_req_pub);

  auto reloc_req_pub =
    Publisher<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      Prefix + fleet_name + "/" + RelocalizationRequestTopicName);
  REQUIRE(reloc_req_pub);

  std::string robot_name = "test_robot";
  uint32_t current_task_id = 1;

  free_fleet::messages::RobotState state;
  state.name = robot_name;
  state.task_id = current_task_id;
  CHECK_NOTHROW(client->send_state(state));

  auto received_states = state_sub->read();
  REQUIRE(!received_states.empty());
  CHECK(std::string(received_states[0]->name) == robot_name);
  CHECK(received_states[0]->task_id == current_task_id);

  MiddlewareMessages_ModeRequest* mode_req_msg =
    MiddlewareMessages_ModeRequest__alloc();
  mode_req_msg->robot_name = dds_string_alloc_and_copy(robot_name);
  mode_req_msg->task_id = ++current_task_id;
  CHECK(mode_req_pub->write(mode_req_msg));

  auto mode_request = client->read_mode_request();
  REQUIRE(mode_request.has_value());
  CHECK(mode_request.value().robot_name == robot_name);
  CHECK(mode_request.value().task_id == current_task_id);

  MiddlewareMessages_NavigationRequest* nav_req_msg =
    MiddlewareMessages_NavigationRequest__alloc();
  nav_req_msg->robot_name = dds_string_alloc_and_copy(robot_name);
  nav_req_msg->task_id = ++current_task_id;
  CHECK(nav_req_pub->write(nav_req_msg));

  auto nav_request = client->read_navigation_request();
  REQUIRE(nav_request.has_value());
  CHECK(nav_request.value().robot_name == robot_name);
  CHECK(nav_request.value().task_id == current_task_id);

  MiddlewareMessages_RelocalizationRequest* reloc_req_msg =
    MiddlewareMessages_RelocalizationRequest__alloc();
  reloc_req_msg->robot_name = dds_string_alloc_and_copy(robot_name);
  reloc_req_msg->task_id = ++current_task_id;
  CHECK(reloc_req_pub->write(reloc_req_msg));

  auto reloc_request = client->read_relocalization_request();
  REQUIRE(reloc_request.has_value());
  CHECK(reloc_request.value().robot_name == robot_name);
  CHECK(reloc_request.value().task_id == current_task_id);

  MiddlewareMessages_ModeRequest_free(mode_req_msg, DDS_FREE_ALL);
  MiddlewareMessages_NavigationRequest_free(nav_req_msg, DDS_FREE_ALL);
  MiddlewareMessages_RelocalizationRequest_free(reloc_req_msg, DDS_FREE_ALL);
  dds_return_t rc = dds_delete(participant);
  REQUIRE(rc == DDS_RETCODE_OK);
}
