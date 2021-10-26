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
#include <free_fleet_cyclonedds/ClientDDSMiddleware.hpp>

#include "src/Publisher.hpp"
#include "src/Subscriber.hpp"
#include "src/messages/convert.hpp"

SCENARIO("Single client with mock subscribers and publishers")
{
  using namespace free_fleet;
  using namespace free_fleet::cyclonedds;
  const int dds_domain = 42;
  const std::string robot_name = "test_robot";
  const std::string fleet_name = "test_fleet";

  auto client = ClientDDSMiddleware::make(dds_domain, fleet_name);
  REQUIRE_FALSE(client == nullptr);

  GIVEN("PauseRequest")
  {
    using namespace free_fleet::messages;

    PauseRequest p("test_robot", 123);

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const messages::PauseRequest& msg)
      {
        received_req = true;
        if (msg == p)
          received_same_req = true;
      };
    client->set_pause_request_callback(std::move(cb));

    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto pause_req_pub = Publisher<MiddlewareMessages_PauseRequest>::make(
      participant,
      &MiddlewareMessages_PauseRequest_desc,
      namespacify(Prefix, fleet_name, PauseRequestTopicName));

    auto dds_msg = convert(p);
    REQUIRE(dds_msg.has_value());

    REQUIRE(pause_req_pub->write(&dds_msg.value()));
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    CHECK(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("ResumeRequest")
  {
    using namespace free_fleet::messages;

    ResumeRequest r("test_robot", 123);

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const messages::ResumeRequest& msg)
      {
        received_req = true;
        if (msg == r)
          received_same_req = true;
      };
    client->set_resume_request_callback(std::move(cb));

    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto resume_req_pub = Publisher<MiddlewareMessages_ResumeRequest>::make(
      participant,
      &MiddlewareMessages_ResumeRequest_desc,
      namespacify(Prefix, fleet_name, ResumeRequestTopicName));

    auto dds_msg = convert(r);
    REQUIRE(dds_msg.has_value());

    REQUIRE(resume_req_pub->write(&dds_msg.value()));
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    CHECK(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("DockRequest")
  {
    using namespace free_fleet::messages;

    DockRequest d("test_robot", 123, "test_dock");

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const messages::DockRequest& msg)
      {
        received_req = true;
        if (msg == d)
          received_same_req = true;
      };
    client->set_dock_request_callback(std::move(cb));

    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto dock_req_pub = Publisher<MiddlewareMessages_DockRequest>::make(
      participant,
      &MiddlewareMessages_DockRequest_desc,
      namespacify(Prefix, fleet_name, DockRequestTopicName));

    auto dds_msg = convert(d);
    REQUIRE(dds_msg.has_value());

    REQUIRE(dock_req_pub->write(&dds_msg.value()));
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    CHECK(received_same_req);

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
    auto cb = [&](const messages::NavigationRequest& msg)
      {
        received_req = true;
        if (msg == n)
          received_same_req = true;
      };
    client->set_navigation_request_callback(std::move(cb));

    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto nav_req_pub = Publisher<MiddlewareMessages_NavigationRequest>::make(
      participant,
      &MiddlewareMessages_NavigationRequest_desc,
      namespacify(Prefix, fleet_name, NavigationRequestTopicName));

    auto dds_msg = convert(n);
    REQUIRE(dds_msg.has_value());
    REQUIRE(nav_req_pub->write(&dds_msg.value()));
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    CHECK(received_same_req);

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
    auto cb = [&](const messages::RelocalizationRequest& msg)
      {
        received_req = true;
        if (msg == reloc)
          received_same_req = true;
      };
    client->set_relocalization_request_callback(std::move(cb));

    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto reloc_req_pub =
      Publisher<MiddlewareMessages_RelocalizationRequest>::make(
      participant,
      &MiddlewareMessages_RelocalizationRequest_desc,
      namespacify(Prefix, fleet_name, RelocalizationRequestTopicName));

    auto dds_msg = convert(reloc);
    REQUIRE(dds_msg.has_value());
    REQUIRE(reloc_req_pub->write(&dds_msg.value()));
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    CHECK(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }

  GIVEN("RobotState")
  {
    using namespace free_fleet::messages;

    rmf_traffic::Time t = std::chrono::steady_clock::now();
    RobotMode m(RobotMode::Mode::Charging, "test_info");
    Location loc("test_map", {1.2, 3.4}, 5.6);
    RobotState s(t, "test_robot", "test_model", std::nullopt, m, 0.9, loc, 321);

    bool received_req = false;
    bool received_same_req = false;
    auto cb = [&](const MiddlewareMessages_RobotState& msg)
      {
        received_req = true;
        auto converted_msg = convert(msg);
        if (converted_msg.has_value() && converted_msg.value() == s)
          received_same_req = true;
      };
    dds_entity_t participant = dds_create_participant(dds_domain, NULL, NULL);
    REQUIRE(participant >= 0);
    auto state_sub = Subscriber<MiddlewareMessages_RobotState>::make(
      participant,
      &MiddlewareMessages_RobotState_desc,
      namespacify(Prefix, fleet_name, StateTopicName),
      std::move(cb));
    REQUIRE_FALSE(state_sub == nullptr);

    client->send_state(s);
    dds_sleepfor(DDS_MSECS(50));

    REQUIRE(received_req);
    REQUIRE(received_same_req);

    dds_return_t rc = dds_delete(participant);
    REQUIRE(rc == DDS_RETCODE_OK);
  }
}
