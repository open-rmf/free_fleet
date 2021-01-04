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
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include "mock_Middleware.hpp"
#include "mock_StatusHandle.hpp"
#include "mock_CommandHandle.hpp"

#include "src/agv/RobotInfo.hpp"
#include "src/requests/RequestInfo.hpp"
#include "src/requests/ModeRequestInfo.hpp"
#include "src/requests/NavigationRequestInfo.hpp"
#include "src/requests/RelocalizationRequestInfo.hpp"

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
    free_fleet::messages::Location next_location {
      0,
      0,
      10.0,
      0.0,
      0.0,
      test_map_name
    };
    auto next_state = initial_state;
    next_state.location = next_location;
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
    free_fleet::messages::Location next_location {
      0,
      0,
      10.0 - 0.5 + 1e-3,
      0.0,
      0.0,
      test_map_name
    };
    auto next_state = initial_state;
    next_state.location = next_location;
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
    free_fleet::messages::Location next_location {
      0,
      0,
      10.0 + 0.5 + 1e-3,
      0.0,
      0.0,
      test_map_name
    };
    auto next_state = initial_state;
    next_state.location = next_location;
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    robot_info->update_state(next_state, next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::Lost);
  }

  GIVEN("Second state updated, on lane, lost")
  {
    free_fleet::messages::Location next_location {
      0,
      0,
      7.0,
      0.0,
      0.0,
      test_map_name
    };
    auto next_state = initial_state;
    next_state.location = next_location;
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    robot_info->update_state(next_state, next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::Lost);
  }

  GIVEN("Second state updated, near center waypoint, on multiple lanes,"
    " not on waypoint, lost")
  {
    free_fleet::messages::Location next_location {
      0,
      0,
      0.0 + 0.5 + 1e-3,
      0.0,
      0.0,
      test_map_name
    };
    auto next_state = initial_state;
    next_state.location = next_location;
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    robot_info->update_state(next_state, next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::Lost);
  }

  GIVEN("Second and third state updated, lost, then found")
  {
    free_fleet::messages::Location second_loc {
      0,
      0,
      0.0 + 0.5 + 1e-3,
      0.0,
      0.0,
      test_map_name
    };
    auto second_state = initial_state;
    second_state.location = second_loc;
    rmf_traffic::Time second_time = std::chrono::steady_clock::now();
    robot_info->update_state(second_state, second_time);

    CHECK(robot_info->state() == second_state);
    CHECK(robot_info->last_updated() == second_time);

    auto second_tracking_estimation = robot_info->tracking_estimation();
    CHECK(second_tracking_estimation.first == TrackingState::Lost);

    free_fleet::messages::Location third_loc {
      0,
      0,
      10.0 - 0.5 + 1e-3,
      0.0,
      0.0,
      test_map_name
    };
    auto third_state = initial_state;
    third_state.location = third_loc;
    rmf_traffic::Time third_time = std::chrono::steady_clock::now();
    robot_info->update_state(third_state, third_time);

    CHECK(robot_info->state() == third_state);
    CHECK(robot_info->last_updated() == third_time);

    auto third_tracking_estimation = robot_info->tracking_estimation();
    CHECK(third_tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(third_tracking_estimation.second == 1);
  }

  GIVEN("Allocated mode request")
  {
    free_fleet::messages::RobotMode pause_mode {
      free_fleet::messages::RobotMode::MODE_PAUSED
    };
    free_fleet::messages::ModeRequest pause_request {
      initial_state.name,
      initial_state.task_id + 1,
      pause_mode,
      {}
    };

    bool request_sent = false;
    auto new_mode_request_info =
      std::make_shared<free_fleet::requests::ModeRequestInfo>(
        pause_request,
        [&](const free_fleet::messages::ModeRequest&){ request_sent = true; },
        std::chrono::steady_clock::now());
    REQUIRE(new_mode_request_info);

    CHECK_NOTHROW(
      robot_info->allocate_task(
        std::dynamic_pointer_cast<free_fleet::requests::RequestInfo>(
          new_mode_request_info)));
  }

  GIVEN("Allocated relocation request")
  {
    free_fleet::messages::Location reloc_loc {
      0,
      0,
      0.0,
      10.0 + 0.5 - 1e-3,
      0.0,
      test_map_name
    };
    free_fleet::messages::RelocalizationRequest reloc_request {
      initial_state.name,
      initial_state.task_id + 1,
      reloc_loc,
      3
    };

    bool request_sent = false;
    auto new_reloc_request_info =
      std::make_shared<free_fleet::requests::RelocalizationRequestInfo>(
        reloc_request,
        [&](const free_fleet::messages::RelocalizationRequest&) { request_sent = true; },
        std::chrono::steady_clock::now());
    REQUIRE(new_reloc_request_info);

    CHECK_NOTHROW(
      robot_info->allocate_task(
        std::dynamic_pointer_cast<free_fleet::requests::RequestInfo>(
          new_reloc_request_info)));
  }

  GIVEN("Allocated navigation request")
  {
    free_fleet::messages::NavigationRequest nav_request {
      initial_state.name,
      initial_state.task_id + 1,
      {}
    };
    free_fleet::messages::Waypoint wp;
    wp.location.level_name = test_map_name;

    auto first_wp = wp;
    first_wp.index = 0;
    first_wp.location.x = 0.0;
    first_wp.location.y = 0.0;
    nav_request.path.push_back(first_wp);

    auto second_wp = wp;
    second_wp.index = 1;
    second_wp.location.x = 10.0;
    nav_request.path.push_back(second_wp);

    auto third_wp = wp;
    third_wp.index = 0;
    third_wp.location.x = 0.0;
    nav_request.path.push_back(third_wp);

    auto forth_wp = wp;
    forth_wp.index = 2;
    forth_wp.location.x = -10.0;
    nav_request.path.push_back(forth_wp);

    bool request_sent = false;
    auto new_nav_request_info =
      std::make_shared<free_fleet::requests::NavigationRequestInfo>(
        nav_request,
        [&](const free_fleet::messages::NavigationRequest&){ request_sent = true; },
        std::chrono::steady_clock::now());
    REQUIRE(new_nav_request_info);

    CHECK_NOTHROW(
      robot_info->allocate_task(
        std::dynamic_pointer_cast<free_fleet::requests::RequestInfo>(
          new_nav_request_info)));
  }

  GIVEN("Allocate navigation request, update multiple states, throughout the path")
  {
    uint32_t new_task_id = initial_state.task_id + 1;
    free_fleet::messages::NavigationRequest nav_request {
      initial_state.name,
      new_task_id,
      {}
    };
    free_fleet::messages::Waypoint wp;
    wp.location.level_name = test_map_name;

    auto first_wp = wp;
    first_wp.index = 0;
    first_wp.location.x = 0.0;
    first_wp.location.y = 0.0;
    nav_request.path.push_back(first_wp);

    auto second_wp = wp;
    second_wp.index = 1;
    second_wp.location.x = 10.0;
    nav_request.path.push_back(second_wp);

    auto third_wp = wp;
    third_wp.index = 0;
    third_wp.location.x = 0.0;
    nav_request.path.push_back(third_wp);

    auto forth_wp = wp;
    forth_wp.index = 2;
    forth_wp.location.x = -10.0;
    nav_request.path.push_back(forth_wp);

    bool request_sent = false;
    auto new_nav_request_info =
      std::make_shared<free_fleet::requests::NavigationRequestInfo>(
        nav_request,
        [&](const free_fleet::messages::NavigationRequest&){ request_sent = true; },
        std::chrono::steady_clock::now());
    REQUIRE(new_nav_request_info);

    CHECK_NOTHROW(
      robot_info->allocate_task(
        std::dynamic_pointer_cast<free_fleet::requests::RequestInfo>(
          new_nav_request_info)));

    // Second state, starting on the path
    auto second_state = initial_state;
    second_state.task_id = new_task_id;
    second_state.location.x = 0.0;
    second_state.location.y = 0.0;
    rmf_traffic::Time second_time = std::chrono::steady_clock::now();
    robot_info->update_state(second_state, second_time);

    // CHECK(robot_info->state() == second_state);
    // CHECK(robot_info->last_updated() == second_time);

    // auto second_tracking_estimation = robot_info->tracking_estimation();
    // CHECK(second_tracking_estimation.first == TrackingState::Lost);
  }
}
