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

#include <string>
#include <memory>
#include <iostream>

#include <rmf_utils/catch.hpp>

#include <free_fleet/messages/NavigationRequest.hpp>

#include <rmf_traffic/Time.hpp>

#include "src/requests/RequestInfo.hpp"
#include "src/requests/NavigationRequestInfo.hpp"

SCENARIO("Test NavigationRequestInfo")
{
  std::string robot_name = "test_robot";
  uint32_t task_id = 1;
  std::string level_name = "test_level";

  free_fleet::messages::Location loc_0 {
    0,
    0,
    0.0,
    0.0,
    0.0,
    level_name
  };
  free_fleet::messages::Waypoint wp_0 {
    0,
    loc_0
  };

  free_fleet::messages::Location loc_1 {
    0,
    0,
    1.0,
    1.0,
    1.0,
    level_name
  };
  free_fleet::messages::Waypoint wp_1 {
    1,
    loc_1
  };

  free_fleet::messages::NavigationRequest nav_request {
    robot_name,
    task_id,
    {wp_0, wp_1}
  };

  bool send_request_called = false;  

  rmf_traffic::Time time_now = std::chrono::steady_clock::now();

  std::shared_ptr<free_fleet::requests::NavigationRequestInfo>
    new_nav_request_info(
      new free_fleet::requests::NavigationRequestInfo(
        nav_request,
        [&](const free_fleet::messages::NavigationRequest&)
  {
    send_request_called = true;
  },
        time_now));
  REQUIRE(new_nav_request_info);

  auto request = new_nav_request_info->request();
  CHECK(request == nav_request);

  auto id = new_nav_request_info->id();
  CHECK(id == task_id);

  new_nav_request_info->send_request();
  CHECK(send_request_called);

  auto request_info =
    std::dynamic_pointer_cast<free_fleet::requests::RequestInfo>(
      new_nav_request_info);
  REQUIRE(request_info);

  auto init_time = request_info->init_time();
  CHECK(init_time == time_now);
  CHECK(!request_info->acknowledged());
  CHECK(!request_info->acknowledged_time().has_value());

  rmf_traffic::Time new_time = std::chrono::steady_clock::now();
  request_info->acknowledged_time(new_time);
  CHECK(request_info->acknowledged());
  CHECK(request_info->acknowledged_time().has_value());
  CHECK(request_info->acknowledged_time().value() == new_time);

  CHECK(request_info->request_type() ==
    free_fleet::requests::RequestInfo::RequestType::NavigationRequest);
}
