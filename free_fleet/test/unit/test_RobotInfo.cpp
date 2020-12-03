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

#include <memory>
#include <iostream>

#include <rmf_utils/catch.hpp>

#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/RobotState.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include "mock_Middleware.hpp"
#include "mock_StatusHandle.hpp"
#include "mock_CommandHandle.hpp"

#include "src/agv/RobotInfo.hpp"

SCENARIO("Verify that a RobotInfo can be created")
{
  free_fleet::messages::RobotMode initial_mode {
    free_fleet::messages::RobotMode::MODE_MOVING
  };
  free_fleet::messages::Location initial_location {
    0,
    0,
    0.0,
    0.0,
    0.0,
    "test_level"
  };
  free_fleet::messages::RobotState initial_state {
    "test_robot",
    "test_model",
    0,
    initial_mode,
    1.0,
    initial_location,
    0
  };
  std::shared_ptr<rmf_traffic::agv::Graph> graph(new rmf_traffic::agv::Graph);

  rmf_traffic::Time time_now = std::chrono::steady_clock::now();

  free_fleet::agv::RobotInfo::SharedPtr robot_info(
    new free_fleet::agv::RobotInfo(
      initial_state,
      graph,
      time_now));

  REQUIRE(robot_info);

  const auto& state = robot_info->state();
  CHECK(initial_state == state);

  // auto ch = std::make_shared<free_fleet::MockCommandHandle>();
  // auto sh = std::make_shared<free_fleet::MockStatusHandle>();
  // auto m = std::make_shared<free_fleet::MockMiddleware>();

  // auto client = free_fleet::agv::Client::make(
  //   "mock_robot_name",
  //   "mock_robot_model",
  //   ch,
  //   sh,
  //   m);
  // REQUIRE(client);
}
