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

#include <rmf_utils/catch.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include <free_fleet_cyclonedds/StandardNames.hpp>
#include <free_fleet_cyclonedds/ServerDDSMiddleware.hpp>

#include "src/Publisher.hpp"
#include "src/Subscriber.hpp"
#include "src/messages/convert.hpp"
#include "MiddlewareMessages.h"

SCENARIO("Single server with mock subscribers and publishers")
{
  using namespace free_fleet;
  using namespace free_fleet::cyclonedds;
  const int dds_domain = 42;
  const std::string robot_name = "test_robot";
  const std::string fleet_name = "test_fleet";

  auto server = ServerDDSMiddleware::make_unique(dds_domain, fleet_name);
  REQUIRE_FALSE(server == nullptr);

  GIVEN("RobotState")
  {
    using namespace free_fleet::messages;

    rmf_traffic::Time t = std::chrono::steady_clock::now();
    RobotMode m(RobotMode::Mode::Charging, "test_info");
    Location loc("test_map", {1.2, 3.4}, 5.6);
    RobotState s(t, "test_robot", "test_model", std::nullopt, m, 0.9, loc, 321);

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const messages::RobotState& msg)
      {
        received_req = true;
        if (msg == s)
          received_same_req = true;
      };
    server->set_robot_state_callback(std::move(cb));

    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto state_pub = Publisher<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      namespacify(Prefix, fleet_name, StateTopicName));
    REQUIRE_FALSE(state_pub == nullptr);

    auto dds_msg = convert(s);
    REQUIRE(dds_msg.has_value());
    REQUIRE(state_pub->write(&dds_msg.value()));
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    CHECK(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("PauseRequest")
  {
    using namespace free_fleet::messages;

    PauseRequest p("test_robot", 123);

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const MiddlewareMessages_PauseRequest& msg)
      {
        received_req = true;
        auto converted_msg = convert(msg);
        if (converted_msg.has_value() && converted_msg.value() == p)
          received_same_req = true;
      };
    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto pause_req_sub = Subscriber<MiddlewareMessages_PauseRequest>::make(
      participant,
      &MiddlewareMessages_PauseRequest_desc,
      namespacify(Prefix, fleet_name, PauseRequestTopicName),
      std::move(cb));
    REQUIRE_FALSE(pause_req_sub == nullptr);

    server->send_pause_request(p);
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    REQUIRE(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ResumeRequest")
  {
    using namespace free_fleet::messages;

    ResumeRequest r("test_robot", 123);

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const MiddlewareMessages_ResumeRequest& msg)
      {
        received_req = true;
        auto converted_msg = convert(msg);
        if (converted_msg.has_value() && converted_msg.value() == r)
          received_same_req = true;
      };
    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto resume_req_sub = Subscriber<MiddlewareMessages_ResumeRequest>::make(
      participant,
      &MiddlewareMessages_ResumeRequest_desc,
      namespacify(Prefix, fleet_name, ResumeRequestTopicName),
      std::move(cb));
    REQUIRE_FALSE(resume_req_sub == nullptr);

    server->send_resume_request(r);
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    REQUIRE(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("DockRequest")
  {
    using namespace free_fleet::messages;

    DockRequest d("test_robot", 123, "test_dock");

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const MiddlewareMessages_DockRequest& msg)
      {
        received_req = true;
        auto converted_msg = convert(msg);
        if (converted_msg.has_value() && converted_msg.value() == d)
          received_same_req = true;
      };
    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto dock_req_sub = Subscriber<MiddlewareMessages_DockRequest>::make(
      participant,
      &MiddlewareMessages_DockRequest_desc,
      namespacify(Prefix, fleet_name, DockRequestTopicName),
      std::move(cb));
    REQUIRE_FALSE(dock_req_sub == nullptr);

    server->send_dock_request(d);
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    REQUIRE(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("NavigationRequest")
  {
    using namespace free_fleet::messages;

    Location loc1("test_map", {1.2, 3.4}, 5.6);
    rmf_traffic::Time t1 = std::chrono::steady_clock::now();
    Waypoint wp1(123, loc1, t1);

    Location loc2("test_map", {3.4, 5.6}, 7.8);
    rmf_traffic::Time t2 = std::chrono::steady_clock::now();
    Waypoint wp2(124, loc2, t2);

    NavigationRequest n("test_robot", 123, {wp1, wp2});

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const MiddlewareMessages_NavigationRequest& msg)
      {
        received_req = true;
        auto converted_msg = convert(msg);
        if (converted_msg.has_value() && converted_msg.value() == n)
          received_same_req = true;
      };
    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto navigation_req_sub =
      Subscriber<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      namespacify(Prefix, fleet_name, NavigationRequestTopicName),
      std::move(cb));
    REQUIRE_FALSE(navigation_req_sub == nullptr);

    server->send_navigation_request(n);
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    REQUIRE(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RelocalizationRequest")
  {
    using namespace free_fleet::messages;

    Location loc("test_map", {1.2, 3.4}, 5.6);
    RelocalizationRequest reloc("test_robot", 123, loc, 321);

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const MiddlewareMessages_RelocalizationRequest& msg)
      {
        received_req = true;
        auto converted_msg = convert(msg);
        if (converted_msg.has_value() && converted_msg.value() == reloc)
          received_same_req = true;
      };
    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto relocalization_req_sub =
      Subscriber<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      namespacify(Prefix, fleet_name, RelocalizationRequestTopicName),
      std::move(cb));
    REQUIRE_FALSE(relocalization_req_sub == nullptr);

    server->send_relocalization_request(reloc);
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    REQUIRE(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }
}
