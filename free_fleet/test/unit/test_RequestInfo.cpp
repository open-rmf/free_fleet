/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <string>
#include <memory>
#include <iostream>

#include <rmf_utils/catch.hpp>


#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include <rmf_traffic/Time.hpp>

#include "src/requests/RequestInfo.hpp"
#include "src/requests/SimpleRequestInfo.hpp"

SCENARIO("Testing request info API")
{
  using RequestInfo = free_fleet::requests::RequestInfo;

  const std::string robot_name = "test_robot";
  const uint32_t initial_task_id = 0;
  auto time_now = std::chrono::steady_clock::now();

  GIVEN("Dock request info")
  {
    using DockRequest = free_fleet::messages::DockRequest;

    DockRequest request {
      robot_name,
      initial_task_id + 1,
      "mock_dock"};

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::requests::SimpleRequestInfo<DockRequest>(
        request,
        [&](const DockRequest&){request_sent = true;},
        time_now));
    
    REQUIRE(request_info);
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto ack_time = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(ack_time));
    CHECK(request_info->acknowledged());
    CHECK(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value() == ack_time);
  }

  GIVEN("Pause request info")
  {
    using PauseRequest = free_fleet::messages::PauseRequest;

    PauseRequest request {
      robot_name,
      initial_task_id + 1};

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::requests::SimpleRequestInfo<PauseRequest>(
        request,
        [&](const PauseRequest&){request_sent = true;},
        time_now));
    
    REQUIRE(request_info);
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto ack_time = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(ack_time));
    CHECK(request_info->acknowledged());
    CHECK(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value() == ack_time);
  }

  GIVEN("Resume request info")
  {
    using ResumeRequest = free_fleet::messages::ResumeRequest;

    ResumeRequest request {
      robot_name,
      initial_task_id + 1};

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::requests::SimpleRequestInfo<ResumeRequest>(
        request,
        [&](const ResumeRequest&){request_sent = true;},
        time_now));
    
    REQUIRE(request_info);
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto ack_time = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(ack_time));
    CHECK(request_info->acknowledged());
    CHECK(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value() == ack_time);
  }

  GIVEN("Relocalization request info")
  {
    using RelocalizationRequest = free_fleet::messages::RelocalizationRequest;

    free_fleet::messages::Location reloc_loc {
      0,
      0,
      0.0,
      10.0 + 0.5 - 1e-3,
      0.0,
      "test_map"};
    RelocalizationRequest request {
      robot_name,
      initial_task_id + 1,
      reloc_loc,
      0};

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::requests::SimpleRequestInfo<RelocalizationRequest>(
        request,
        [&](const RelocalizationRequest&){request_sent = true;},
        time_now));

    REQUIRE(request_info);
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto ack_time = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(ack_time));
    CHECK(request_info->acknowledged());
    CHECK(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value() == ack_time);
  }

  GIVEN("Navigation request info")
  {
    using NavigationRequest = free_fleet::messages::NavigationRequest;

    NavigationRequest request {
      robot_name,
      initial_task_id + 1,
      {}
    };
    free_fleet::messages::Waypoint wp;
    wp.location.level_name = "test_map";

    auto first_wp = wp;
    first_wp.index = 0;
    first_wp.location.x = 0.0;
    first_wp.location.y = 0.0;
    request.path.push_back(first_wp);

    auto second_wp = wp;
    second_wp.index = 1;
    second_wp.location.x = 10.0;
    request.path.push_back(second_wp);

    auto third_wp = wp;
    third_wp.index = 0;
    third_wp.location.x = 0.0;
    request.path.push_back(third_wp);

    auto forth_wp = wp;
    forth_wp.index = 2;
    forth_wp.location.x = -10.0;
    request.path.push_back(forth_wp);

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::requests::SimpleRequestInfo<NavigationRequest>(
        request,
        [&](const NavigationRequest&){request_sent = true;},
        time_now));

    REQUIRE(request_info);
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto ack_time = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(ack_time));
    CHECK(request_info->acknowledged());
    CHECK(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value() == ack_time);
  }
}
