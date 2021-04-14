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

#include "src/RequestInfo.hpp"

SCENARIO("Testing request info API")
{
  const std::string robot_name = "test_robot";
  const uint32_t initial_task_id = 0;

  using BaseRequestInfo = free_fleet::requests::BaseRequestInfo;

  auto time_now = std::chrono::steady_clock::now();

  GIVEN("Pause request info")
  {
    using PauseRequest = free_fleet::messages::PauseRequest;

    PauseRequest request {
      robot_name,
      initial_task_id + 1};

    bool request_sent = false;
    std::shared_ptr<free_fleet::requests::RequestInfo<PauseRequest>>
      request_info =
        std::make_shared<free_fleet::requests::RequestInfo<PauseRequest>>(
          BaseRequestInfo::RequestType::PauseRequest,
          request,
          [&](const PauseRequest&) {request_sent = true;},
          time_now);
    
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->request_type() ==
      BaseRequestInfo::RequestType::PauseRequest);
    CHECK(request_info->id() == request.task_id);
    CHECK(request_info->request() == request);

    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto new_time_now = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(new_time_now));
    CHECK(request_info->acknowledged());
    REQUIRE(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value()  == new_time_now);
  }

  GIVEN("Resume request info")
  {
    using ResumeRequest = free_fleet::messages::ResumeRequest;

    ResumeRequest request {
      robot_name,
      initial_task_id + 1};

    bool request_sent = false;
    std::shared_ptr<free_fleet::requests::RequestInfo<ResumeRequest>>
      request_info =
        std::make_shared<free_fleet::requests::RequestInfo<ResumeRequest>>(
          BaseRequestInfo::RequestType::ResumeRequest,
          request,
          [&](const ResumeRequest&) {request_sent = true;},
          time_now);
    
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->request_type() ==
      BaseRequestInfo::RequestType::ResumeRequest);
    CHECK(request_info->id() == request.task_id);
    CHECK(request_info->request() == request);

    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto new_time_now = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(new_time_now));
    CHECK(request_info->acknowledged());
    REQUIRE(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value()  == new_time_now);
  }

  GIVEN("Dock request info")
  {
    using DockRequest = free_fleet::messages::DockRequest;

    DockRequest request {
      robot_name,
      initial_task_id + 1,
      "test_dock"};

    bool request_sent = false;
    std::shared_ptr<free_fleet::requests::RequestInfo<DockRequest>>
      request_info =
        std::make_shared<free_fleet::requests::RequestInfo<DockRequest>>(
          BaseRequestInfo::RequestType::DockRequest,
          request,
          [&](const DockRequest&) {request_sent = true;},
          time_now);
    
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->request_type() ==
      BaseRequestInfo::RequestType::DockRequest);
    CHECK(request_info->id() == request.task_id);
    CHECK(request_info->request() == request);

    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto new_time_now = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(new_time_now));
    CHECK(request_info->acknowledged());
    REQUIRE(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value()  == new_time_now);
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
    std::shared_ptr<free_fleet::requests::RequestInfo<RelocalizationRequest>>
      request_info =
        std::make_shared<free_fleet::requests::RequestInfo<RelocalizationRequest>>(
          BaseRequestInfo::RequestType::RelocalizationRequest,
          request,
          [&](const RelocalizationRequest&) {request_sent = true;},
          time_now);
    
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->request_type() ==
      BaseRequestInfo::RequestType::RelocalizationRequest);
    CHECK(request_info->id() == request.task_id);
    CHECK(request_info->request() == request);

    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto new_time_now = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(new_time_now));
    CHECK(request_info->acknowledged());
    REQUIRE(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value()  == new_time_now);
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
    std::shared_ptr<free_fleet::requests::RequestInfo<NavigationRequest>>
      request_info =
        std::make_shared<free_fleet::requests::RequestInfo<NavigationRequest>>(
          BaseRequestInfo::RequestType::NavigationRequest,
          request,
          [&](const NavigationRequest&) {request_sent = true;},
          time_now);
    
    CHECK(request_info->init_time() == time_now);
    CHECK(!request_info->acknowledged());
    CHECK(!request_info->acknowledged_time().has_value());
    CHECK(request_info->request_type() ==
      BaseRequestInfo::RequestType::NavigationRequest);
    CHECK(request_info->id() == request.task_id);
    CHECK(request_info->request() == request);

    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto new_time_now = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledged_time(new_time_now));
    CHECK(request_info->acknowledged());
    REQUIRE(request_info->acknowledged_time().has_value());
    CHECK(request_info->acknowledged_time().value()  == new_time_now);
  }
}
