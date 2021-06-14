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

#include <free_fleet/messages/RobotState.hpp>

#include <free_fleet/manager/RobotInfo.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include "src/manager/internal_RobotInfo.hpp"

SCENARIO("Make RobotInfo")
{
  const std::string test_map_name = "test_level";
  free_fleet::messages::RobotMode initial_mode {
    free_fleet::messages::RobotMode::MODE_IDLE,
    ""
  };
  free_fleet::messages::Location initial_location {
    0,
    0,
    0.0,
    0.0,
    0.0,
    test_map_name
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

  rmf_traffic::Time initial_time = std::chrono::steady_clock::now();

  GIVEN("Valid traffic graph")
  {
    auto robot_info =
      free_fleet::manager::RobotInfo::Implementation::make(
        initial_state,
        graph,
        initial_time);
    CHECK(robot_info);
  }

  GIVEN("Invalid traffic graph")
  {
    auto robot_info =
      free_fleet::manager::RobotInfo::Implementation::make(
        initial_state,
        nullptr,
        initial_time);
    CHECK(!robot_info);
  }

  GIVEN("Empty robot name")
  {
    initial_state.name = "";
    auto robot_info =
      free_fleet::manager::RobotInfo::Implementation::make(
        initial_state,
        graph,
        initial_time);
    CHECK(!robot_info);
  }

  GIVEN("Empty robot model")
  {
    initial_state.model = "";
    auto robot_info =
      free_fleet::manager::RobotInfo::Implementation::make(
        initial_state,
        graph,
        initial_time);
    CHECK(!robot_info);
  }
}
