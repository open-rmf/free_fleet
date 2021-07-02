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

#include <free_fleet/Types.hpp>
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include <rmf_traffic/Time.hpp>

#include "src/manager/requests/RequestInfo.hpp"
#include "src/manager/requests/SimpleRequestInfo.hpp"

SCENARIO("Testing request info API")
{
  using RequestInfo = free_fleet::manager::RequestInfo;

  const std::string robot_name = "test_robot";
  const free_fleet::TaskId initial_task_id = 0;
  auto time_now = std::chrono::steady_clock::now();

  GIVEN("Dock request info")
  {
    using DockRequest = free_fleet::messages::DockRequest;

    DockRequest request(
      robot_name,
      initial_task_id + 1,
      "mock_dock");

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<DockRequest>(
        request,
        [&](const DockRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();}));
    
    REQUIRE(request_info);
    CHECK(request_info->init_time() >= time_now);
    CHECK(!request_info->acknowledged().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto time_before_ack = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledge_request());
    auto ack_time = request_info->acknowledged();
    REQUIRE(ack_time.has_value());
    CHECK(ack_time.value() > time_now);
    CHECK(ack_time.value() > request_info->init_time());
    CHECK(ack_time.value() > time_before_ack);
  }

  GIVEN("Pause request info")
  {
    using PauseRequest = free_fleet::messages::PauseRequest;

    PauseRequest request(
      robot_name,
      initial_task_id + 1);

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<PauseRequest>(
        request,
        [&](const PauseRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    
    REQUIRE(request_info);
    CHECK(request_info->init_time() >= time_now);
    CHECK(!request_info->acknowledged().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto time_before_ack = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledge_request());
    auto ack_time = request_info->acknowledged();
    REQUIRE(ack_time.has_value());
    CHECK(ack_time.value() > time_now);
    CHECK(ack_time.value() > request_info->init_time());
    CHECK(ack_time.value() > time_before_ack);
  }

  GIVEN("Resume request info")
  {
    using ResumeRequest = free_fleet::messages::ResumeRequest;

    ResumeRequest request(
      robot_name,
      initial_task_id + 1);

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<ResumeRequest>(
        request,
        [&](const ResumeRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    
    REQUIRE(request_info);
    CHECK(request_info->init_time() >= time_now);
    CHECK(!request_info->acknowledged().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto time_before_ack = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledge_request());
    auto ack_time = request_info->acknowledged();
    REQUIRE(ack_time.has_value());
    CHECK(ack_time.value() > time_now);
    CHECK(ack_time.value() > request_info->init_time());
    CHECK(ack_time.value() > time_before_ack);
  }

  GIVEN("Relocalization request info")
  {
    using RelocalizationRequest = free_fleet::messages::RelocalizationRequest;

    free_fleet::messages::Location reloc_loc(
      "test_map",
      {0.0, 10.0 + 0.5 - 1e-3},
      0.0);
    RelocalizationRequest request(
      robot_name,
      initial_task_id + 1,
      reloc_loc,
      0);

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<RelocalizationRequest>(
        request,
        [&](const RelocalizationRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    
    REQUIRE(request_info);
    CHECK(request_info->init_time() >= time_now);
    CHECK(!request_info->acknowledged().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto time_before_ack = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledge_request());
    auto ack_time = request_info->acknowledged();
    REQUIRE(ack_time.has_value());
    CHECK(ack_time.value() > time_now);
    CHECK(ack_time.value() > request_info->init_time());
    CHECK(ack_time.value() > time_before_ack);
  }

  GIVEN("Navigation request info")
  {
    using NavigationRequest = free_fleet::messages::NavigationRequest;
    using Waypoint = free_fleet::messages::Waypoint;
    using Location = free_fleet::messages::Location;

    Waypoint first_wp(0, Location("test_map", {0.0, 0.0}, 0.0));
    Waypoint second_wp(1, Location("test_map", {10.0, 0.0}, 0.0));
    Waypoint third_wp(0, Location("test_map", {0.0, 0.0}, 0.0));
    Waypoint forth_wp(2, Location("test_map", {-10.0, 0.0}, 0.0));

    NavigationRequest request(
      robot_name,
      initial_task_id + 1,
      {first_wp, second_wp, third_wp, forth_wp});
    
    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<NavigationRequest>(
        request,
        [&](const NavigationRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    
    REQUIRE(request_info);
    CHECK(request_info->init_time() >= time_now);
    CHECK(!request_info->acknowledged().has_value());
    CHECK(request_info->id() == initial_task_id + 1);
    CHECK_NOTHROW(request_info->send_request());
    CHECK(request_sent);

    auto time_before_ack = std::chrono::steady_clock::now();
    CHECK_NOTHROW(request_info->acknowledge_request());
    auto ack_time = request_info->acknowledged();
    REQUIRE(ack_time.has_value());
    CHECK(ack_time.value() > time_now);
    CHECK(ack_time.value() > request_info->init_time());
    CHECK(ack_time.value() > time_before_ack);
  }
}
