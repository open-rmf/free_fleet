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

#include <free_fleet/messages/ModeRequest.hpp>

#include <rmf_traffic/Time.hpp>

#include "src/requests/RequestInfo.hpp"
#include "src/requests/ModeRequestInfo.hpp"

SCENARIO("Test ModeRequestInfo")
{
  std::string robot_name = "test_robot";
  uint32_t task_id = 1;

  free_fleet::messages::RobotMode pause_mode {
    free_fleet::messages::RobotMode::MODE_PAUSED,
    ""
  };
  free_fleet::messages::ModeRequest pause_request {
    robot_name,
    task_id,
    pause_mode,
    {}
  };

  bool send_request_called = false;

  rmf_traffic::Time time_now = std::chrono::steady_clock::now();

  std::shared_ptr<free_fleet::requests::ModeRequestInfo> new_mode_request_info(
    new free_fleet::requests::ModeRequestInfo(
      pause_request,
      [&](const free_fleet::messages::ModeRequest&)
  {
    send_request_called = true;
  },
      time_now));
  REQUIRE(new_mode_request_info);

  auto request = new_mode_request_info->request();
  CHECK(request == pause_request);

  auto id = new_mode_request_info->id();
  CHECK(id == task_id);

  new_mode_request_info->send_request();
  CHECK(send_request_called);

  auto request_info =
    std::dynamic_pointer_cast<free_fleet::requests::RequestInfo>(
      new_mode_request_info);
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
    free_fleet::requests::RequestInfo::RequestType::ModeRequest);
}

