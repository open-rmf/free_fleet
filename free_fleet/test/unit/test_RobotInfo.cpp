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

SCENARIO("Tests RobotInfo API")
{
  const std::string test_map_name = "test_level";
  free_fleet::messages::RobotMode initial_mode {
    free_fleet::messages::RobotMode::MODE_IDLE
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
  graph->add_waypoint(test_map_name, {0, 0});
  graph->add_waypoint(test_map_name, {10, 0});
  graph->add_waypoint(test_map_name, {-10, 0});
  graph->add_waypoint(test_map_name, {0, 10});
  graph->add_waypoint(test_map_name, {0, -10});
  REQUIRE(graph->num_waypoints() == 5);
  graph->add_lane(0, 1);
  graph->add_lane(1, 0);
  graph->add_lane(0, 2);
  graph->add_lane(2, 0);
  graph->add_lane(0, 3);
  graph->add_lane(3, 0);
  graph->add_lane(0, 4);
  graph->add_lane(4, 0);
  REQUIRE(graph->num_lanes() == 8);

  rmf_traffic::Time initial_time = std::chrono::steady_clock::now();

  free_fleet::agv::RobotInfo::SharedPtr robot_info(
    new free_fleet::agv::RobotInfo(
      initial_state,
      graph,
      initial_time));
  REQUIRE(robot_info);

  using TrackingState = free_fleet::agv::RobotInfo::TrackingState;

  GIVEN("Starting state to RobotInfo")
  {
    const auto& state = robot_info->state();
    CHECK(initial_state == state);
    
    auto last_updated_time = robot_info->last_updated();
    CHECK(last_updated_time == initial_time);

    auto first_found_time = robot_info->first_found();
    CHECK(first_found_time == initial_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(tracking_estimation.second == 0);
  }

  GIVEN("Second state updated, on waypoint")
  {
    free_fleet::messages::RobotMode next_mode {
      free_fleet::messages::RobotMode::MODE_MOVING
    };
    free_fleet::messages::Location next_location {
      0,
      0,
      10.0,
      0.0,
      0.0,
      test_map_name
    };
    free_fleet::messages::RobotState next_state {
      initial_state.name,
      initial_state.model,
      initial_state.task_id,
      next_mode,
      1.0,
      next_location,
      0
    };
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    robot_info->update_state(next_state, next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(tracking_estimation.second == 1);
  }

  GIVEN("Second state updated, near waypoint")
  {
    free_fleet::messages::RobotMode next_mode {
      free_fleet::messages::RobotMode::MODE_MOVING
    };
    free_fleet::messages::Location next_location {
      0,
      0,
      10.0 - 0.5 + 1e-3,
      0.0,
      0.0,
      test_map_name
    };
    free_fleet::messages::RobotState next_state {
      initial_state.name,
      initial_state.model,
      initial_state.task_id,
      next_mode,
      1.0,
      next_location,
      0
    };
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    robot_info->update_state(next_state, next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(tracking_estimation.second == 1);
  }

  GIVEN("Second state updated, near waypoint but outside threshold")
  {
    free_fleet::messages::RobotMode next_mode {
      free_fleet::messages::RobotMode::MODE_MOVING
    };
    free_fleet::messages::Location next_location {
      0,
      0,
      10.0 + 0.5 + 1e-3,
      0.0,
      0.0,
      test_map_name
    };
    free_fleet::messages::RobotState next_state {
      initial_state.name,
      initial_state.model,
      initial_state.task_id,
      next_mode,
      1.0,
      next_location,
      0
    };
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    robot_info->update_state(next_state, next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::Lost);
  }

  GIVEN("Second state updated, on lane")
  {
    free_fleet::messages::RobotMode next_mode {
      free_fleet::messages::RobotMode::MODE_MOVING
    };
    free_fleet::messages::Location next_location {
      0,
      0,
      7.0,
      0.0,
      0.0,
      test_map_name
    };
    free_fleet::messages::RobotState next_state {
      initial_state.name,
      initial_state.model,
      initial_state.task_id,
      next_mode,
      1.0,
      next_location,
      0
    };
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    robot_info->update_state(next_state, next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::Lost);
  }

  GIVEN("Second state updated, near center waypoint, on multiple lanes")
  {
    free_fleet::messages::RobotMode next_mode {
      free_fleet::messages::RobotMode::MODE_MOVING
    };
    free_fleet::messages::Location next_location {
      0,
      0,
      0.0,
      0.0 + 0.5 - 1e-3,
      0.0,
      test_map_name
    };
    free_fleet::messages::RobotState next_state {
      initial_state.name,
      initial_state.model,
      initial_state.task_id,
      next_mode,
      1.0,
      next_location,
      0
    };
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    robot_info->update_state(next_state, next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(tracking_estimation.second == 0);
  }

  GIVEN("Allocated mode request, state updated")
  {
    
  }

}
