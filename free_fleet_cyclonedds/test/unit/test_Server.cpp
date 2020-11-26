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

SCENARIO("Single server with mock subscribers and publishers")
{
  using namespace free_fleet::cyclonedds;
  int dds_domain = 42;
  std::string fleet_name = "test_fleet";

  auto server = CycloneDDSMiddleware::make_server(dds_domain, fleet_name);
  REQUIRE(server);

  // Create a DDS participant
  dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
  REQUIRE(participant >= 0);

  auto state_pub = Publisher<MiddlewareMessages_RobotState>::make(
    participant,
    &MiddlewareMessages_RobotState_desc,
    Prefix + fleet_name + "/" + StateTopicName);
  REQUIRE(state_pub);

  auto mode_req_sub = Subscriber<MiddlewareMessages_ModeRequest>::make(
    participant,
    &MiddlewareMessages_ModeRequest_desc,
    Prefix + fleet_name + "/" + ModeRequestTopicName);
  REQUIRE(mode_req_sub);

  auto nav_req_sub = Subscriber<MiddlewareMessages_NavigationRequest>::make(
    participant,
    &MiddlewareMessages_NavigationRequest_desc,
    Prefix + fleet_name + "/" + NavigationRequestTopicName);
  REQUIRE(nav_req_sub);

  auto reloc_req_sub =
    Subscriber<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      Prefix + fleet_name + "/" + RelocalizationRequestTopicName);
  REQUIRE(reloc_req_sub);

  std::string robot_name = "test_robot";
  uint32_t current_task_id = 1;

  MiddlewareMessages_RobotState* state_msg =
    MiddlewareMessages_RobotState__alloc();
  state_msg->name = dds_string_alloc_and_copy(robot_name);
  state_msg->task_id = current_task_id;
  CHECK(state_pub->write(state_msg));

  auto states = server->read_states();
  REQUIRE(!states.empty());
  CHECK(states[0].name == robot_name);
  CHECK(states[0].task_id == current_task_id);

  free_fleet::messages::ModeRequest mode_request;
  mode_request.robot_name = robot_name;
  mode_request.task_id = ++current_task_id;
  CHECK_NOTHROW(server->send_mode_request(mode_request));

  auto received_mode_request = mode_req_sub->read();
  REQUIRE(!received_mode_request.empty());
  CHECK(std::string(received_mode_request[0]->robot_name) == robot_name);
  CHECK(received_mode_request[0]->task_id == current_task_id);

  free_fleet::messages::NavigationRequest nav_request;
  nav_request.robot_name = robot_name;
  nav_request.task_id = ++current_task_id;
  CHECK_NOTHROW(server->send_navigation_request(nav_request));

  auto received_nav_request = nav_req_sub->read();
  REQUIRE(!received_nav_request.empty());
  CHECK(std::string(received_nav_request[0]->robot_name) == robot_name);
  CHECK(received_nav_request[0]->task_id == current_task_id);

  free_fleet::messages::RelocalizationRequest reloc_request;
  reloc_request.robot_name = robot_name;
  reloc_request.task_id = ++current_task_id;
  CHECK_NOTHROW(server->send_relocalization_request(reloc_request));

  auto received_reloc_request = reloc_req_sub->read();
  REQUIRE(!received_reloc_request.empty());
  CHECK(std::string(received_reloc_request[0]->robot_name) == robot_name);
  CHECK(received_reloc_request[0]->task_id == current_task_id);

  MiddlewareMessages_RobotState_free(state_msg, DDS_FREE_ALL);
}
