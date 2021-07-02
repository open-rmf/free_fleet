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
#include <free_fleet/messages/DockRequest.hpp>
#include <free_fleet/messages/PauseRequest.hpp>
#include <free_fleet/messages/ResumeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

#include <free_fleet/manager/RobotInfo.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include "src/manager/internal_RobotInfo.hpp"

#include "src/manager/requests/RequestInfo.hpp"
#include "src/manager/requests/SimpleRequestInfo.hpp"

void update_and_check_tracking_state(
  const std::shared_ptr<free_fleet::manager::RobotInfo>& robot_info,
  const free_fleet::messages::RobotState& new_state,
  free_fleet::manager::RobotInfo::TrackingState desired_tracking_state,
  std::size_t desired_tracking_index)
{
  rmf_traffic::Time update_time = std::chrono::steady_clock::now();

  free_fleet::manager::RobotInfo::Implementation::update_state(
    *robot_info,
    new_state,
    update_time);
  CHECK(robot_info->state() == new_state);
  CHECK(robot_info->last_updated() == update_time);

  auto tracking_estimation = robot_info->tracking_estimation();
  CHECK(tracking_estimation.first == desired_tracking_state);
  
  if (desired_tracking_state !=
    free_fleet::manager::RobotInfo::TrackingState::Lost)
  {
    CHECK(tracking_estimation.second == desired_tracking_index);
  }
}

SCENARIO("Tests RobotInfo API")
{
  const std::string test_map_name = "test_level";
  free_fleet::messages::RobotMode initial_mode(
    free_fleet::messages::RobotMode::Mode::Idle);
  free_fleet::messages::Location initial_location(
    test_map_name, {0.0, 0.0}, 0.0);
  free_fleet::messages::RobotState initial_state(
    std::chrono::steady_clock::now(),
    "test_robot",
    "test_model",
    0,
    initial_mode,
    1.0,
    initial_location,
    0);

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

  auto robot_info =
    free_fleet::manager::RobotInfo::Implementation::make(
      initial_state,
      graph,
      initial_time);
  REQUIRE(robot_info);

  using TrackingState = free_fleet::manager::RobotInfo::TrackingState;

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
    free_fleet::messages::Location next_location(
      test_map_name,
      {10.0, 0.0},
      0.0);
    free_fleet::messages::RobotState next_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      initial_state.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      next_location,
      initial_state.target_path_index());

    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    free_fleet::manager::RobotInfo::Implementation::update_state(
      *robot_info,
      next_state,
      next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(tracking_estimation.second == 1);
  }

  GIVEN("Second state updated, near waypoint")
  {
    free_fleet::messages::Location next_location(
      test_map_name,
      {10.0 - 0.5 + 1e-3, 0.0},
      0.0);
    free_fleet::messages::RobotState next_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      initial_state.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      next_location,
      initial_state.target_path_index());
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    free_fleet::manager::RobotInfo::Implementation::update_state(
      *robot_info,
      next_state,
      next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(tracking_estimation.second == 1);
  }

  GIVEN("Second state updated, near waypoint but outside threshold")
  {
    free_fleet::messages::Location next_location(
      test_map_name,
      {10.0 + 0.5 + 1e-3, 0.0},
      0.0);
    free_fleet::messages::RobotState next_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      initial_state.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      next_location,
      initial_state.target_path_index());
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    free_fleet::manager::RobotInfo::Implementation::update_state(
      *robot_info,
      next_state,
      next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::Lost);
  }

  GIVEN("Second state updated, on lane, lost")
  {
    free_fleet::messages::Location next_location(
      test_map_name,
      {7.0, 0.0},
      0.0);
    free_fleet::messages::RobotState next_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      initial_state.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      next_location,
      initial_state.target_path_index());
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    free_fleet::manager::RobotInfo::Implementation::update_state(
      *robot_info,
      next_state,
      next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::Lost);
  }

  GIVEN("Second state updated, near center waypoint, on multiple lanes,"
    " not on waypoint, lost")
  {
    free_fleet::messages::Location next_location(
      test_map_name,
      {0.0 + 0.5 + 1e-3, 0.0},
      0.0);
    free_fleet::messages::RobotState next_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      initial_state.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      next_location,
      initial_state.target_path_index());
    rmf_traffic::Time next_time = std::chrono::steady_clock::now();
    free_fleet::manager::RobotInfo::Implementation::update_state(
      *robot_info,
      next_state,
      next_time);

    CHECK(robot_info->state() == next_state);
    CHECK(robot_info->first_found() == initial_time);
    CHECK(robot_info->last_updated() == next_time);

    auto tracking_estimation = robot_info->tracking_estimation();
    CHECK(tracking_estimation.first == TrackingState::Lost);
  }

  GIVEN("Second and third state updated, lost, then found")
  {
    free_fleet::messages::Location second_loc(
      test_map_name,
      {0.0 + 0.5 + 1e-3, 0.0},
      0.0);
    free_fleet::messages::RobotState second_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      initial_state.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      second_loc,
      initial_state.target_path_index());
    rmf_traffic::Time second_time = std::chrono::steady_clock::now();
    free_fleet::manager::RobotInfo::Implementation::update_state(
      *robot_info,
      second_state,
      second_time);

    CHECK(robot_info->state() == second_state);
    CHECK(robot_info->last_updated() == second_time);

    auto second_tracking_estimation = robot_info->tracking_estimation();
    CHECK(second_tracking_estimation.first == TrackingState::Lost);

    free_fleet::messages::Location third_loc(
      test_map_name,
      {10.0 - 0.5 + 1e-3, 0.0},
      0.0);
    free_fleet::messages::RobotState third_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      initial_state.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      third_loc,
      initial_state.target_path_index());
    rmf_traffic::Time third_time = std::chrono::steady_clock::now();
    free_fleet::manager::RobotInfo::Implementation::update_state(
      *robot_info,
      third_state,
      third_time);

    CHECK(robot_info->state() == third_state);
    CHECK(robot_info->last_updated() == third_time);

    auto third_tracking_estimation = robot_info->tracking_estimation();
    CHECK(third_tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(third_tracking_estimation.second == 1);
  }

  GIVEN("Allocated dock request")
  {
    using RequestInfo = free_fleet::manager::RequestInfo;
    using DockRequest = free_fleet::messages::DockRequest;

    REQUIRE(initial_state.task_id().has_value());
    DockRequest request(
      initial_state.name(),
      initial_state.task_id().value() + 1,
      "mock_dock");

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<DockRequest>(
        request,
        [&](const DockRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    REQUIRE(request_info);

    CHECK_NOTHROW(
      free_fleet::manager::RobotInfo::Implementation::get(
        *robot_info).allocate_task(request_info));
  }

  GIVEN("Allocated pause request")
  {
    using RequestInfo = free_fleet::manager::RequestInfo;
    using PauseRequest = free_fleet::messages::PauseRequest;

    REQUIRE(initial_state.task_id().has_value());
    PauseRequest request(
      initial_state.name(),
      initial_state.task_id().value() + 1);

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<PauseRequest>(
        request,
        [&](const PauseRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    REQUIRE(request_info);

    CHECK_NOTHROW(
      free_fleet::manager::RobotInfo::Implementation::get(
        *robot_info).allocate_task(request_info));
  }

  GIVEN("Allocated resume request")
  {
    using RequestInfo = free_fleet::manager::RequestInfo;
    using ResumeRequest = free_fleet::messages::ResumeRequest;

    REQUIRE(initial_state.task_id().has_value());
    ResumeRequest request(
      initial_state.name(),
      initial_state.task_id().value() + 1);

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<ResumeRequest>(
        request,
        [&](const ResumeRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    REQUIRE(request_info);

    CHECK_NOTHROW(
      free_fleet::manager::RobotInfo::Implementation::get(
        *robot_info).allocate_task(request_info));
  }

  GIVEN("Allocated relocation request")
  {
    using RequestInfo = free_fleet::manager::RequestInfo;
    using RelocalizationRequest = free_fleet::messages::RelocalizationRequest;
 
    free_fleet::messages::Location reloc_loc(
      test_map_name,
      {10.0 + 0.5 - 1e-3, 0.0},
      0.0);
    REQUIRE(initial_state.task_id().has_value());
    RelocalizationRequest request(
      initial_state.name(),
      initial_state.task_id().value() + 1,
      reloc_loc,
      3);

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<RelocalizationRequest>(
        request,
        [&](const RelocalizationRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    REQUIRE(request_info);

    CHECK_NOTHROW(
      free_fleet::manager::RobotInfo::Implementation::get(
        *robot_info).allocate_task(request_info));
  }

  GIVEN("Allocated navigation request")
  {
    using RequestInfo = free_fleet::manager::RequestInfo;
    using NavigationRequest = free_fleet::messages::NavigationRequest;
    using Location = free_fleet::messages::Location;
    using Waypoint = free_fleet::messages::Waypoint;

    Waypoint first_wp(0, Location(test_map_name, {0.0, 0.0}, 0.0));
    Waypoint second_wp(1, Location(test_map_name, {10.0, 0.0}, 0.0));
    Waypoint third_wp(0, Location(test_map_name, {0.0, 0.0}, 0.0));
    Waypoint forth_wp(3, Location(test_map_name, {0.0, 10.0}, 0.0));

    REQUIRE(initial_state.task_id().has_value());
    NavigationRequest request(
      initial_state.name(),
      initial_state.task_id().value() + 1,
      {first_wp, second_wp, third_wp, forth_wp});

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<NavigationRequest>(
        request,
        [&](const NavigationRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    REQUIRE(request_info);

    CHECK_NOTHROW(
      free_fleet::manager::RobotInfo::Implementation::get(
        *robot_info).allocate_task(request_info));
  }

  GIVEN("Allocate navigation request, update multiple states, throughout the path")
  {
    using RequestInfo = free_fleet::manager::RequestInfo;
    using NavigationRequest = free_fleet::messages::NavigationRequest;
    using Location = free_fleet::messages::Location;
    using Waypoint = free_fleet::messages::Waypoint;
    using RobotState = free_fleet::messages::RobotState;

    Waypoint first_wp(0, Location(test_map_name, {0.0, 0.0}, 0.0));
    Waypoint second_wp(1, Location(test_map_name, {10.0, 0.0}, 0.0));
    Waypoint third_wp(0, Location(test_map_name, {0.0, 0.0}, 0.0));
    Waypoint forth_wp(3, Location(test_map_name, {0.0, 10.0}, 0.0));

    REQUIRE(initial_state.task_id().has_value());
    NavigationRequest request(
      initial_state.name(),
      initial_state.task_id().value() + 1,
      {first_wp, second_wp, third_wp, forth_wp});

    bool request_sent = false;
    std::shared_ptr<RequestInfo> request_info(
      new free_fleet::manager::SimpleRequestInfo<NavigationRequest>(
        request,
        [&](const NavigationRequest&){request_sent = true;},
        [](){return std::chrono::steady_clock::now();})); 
    REQUIRE(request_info);

    CHECK_NOTHROW(
      free_fleet::manager::RobotInfo::Implementation::get(
        *robot_info).allocate_task(request_info));

    // Second state, starting on the path
    RobotState second_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {0.0, 0.0}, 0.0),
      initial_state.target_path_index());
    update_and_check_tracking_state(
      robot_info,
      second_state,
      TrackingState::OnWaypoint,
      0);

    // Third state, on the lane
    RobotState third_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {0.0 + 0.5 + 1e-3, 0.0}, 0.0),
      1);
    update_and_check_tracking_state(
      robot_info,
      third_state,
      TrackingState::OnLane,
      0);

    // Forth state, on the lane
    RobotState forth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {10.0 - 0.5 - 1e-3, 0.0}, 0.0),
      1);
    update_and_check_tracking_state(
      robot_info,
      forth_state,
      TrackingState::OnLane,
      0);

    // Fifth state, on waypoint
    RobotState fifth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {10.0 - 0.5 + 1e-3, 1e-3}, 0.0),
      1);
    update_and_check_tracking_state(
      robot_info,
      fifth_state,
      TrackingState::OnWaypoint,
      1);

    // Sixth state, on waypoint
    RobotState sixth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {10.0, 0.5 - 1e-3}, 0.0),
      2);
    update_and_check_tracking_state(
      robot_info,
      sixth_state,
      TrackingState::OnWaypoint,
      1);

    // Seventh state, on lane, a bit lost near waypoint, but with enough
    // information to get the lane
    RobotState seventh_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {10.0, 0.5 + 1e-3}, 0.0),
      2);
    update_and_check_tracking_state(
      robot_info,
      seventh_state,
      TrackingState::OnLane,
      1);
    
    // Eigth state, on lane, on previous waypoint
    RobotState eighth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {10.0, 0.5 - 1e-3}, 0.0),
      2);
    update_and_check_tracking_state(
      robot_info,
      eighth_state,
      TrackingState::OnWaypoint,
      1);

    // Ninth state, on lane
    RobotState ninth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {10.0 - 0.5 - 1e-3, 0.0}, 0.0),
      2);
    update_and_check_tracking_state(
      robot_info,
      ninth_state,
      TrackingState::OnLane,
      1);

    // Tenth state, on waypoint
    RobotState tenth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {0.5 - 1e-3, 0.0}, 0.0),
      3);
    update_and_check_tracking_state(
      robot_info,
      tenth_state,
      TrackingState::OnWaypoint,
      0);
    
    // Eleventh state, on lane
    RobotState eleventh_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {0.0, 0.5 + 1e-3}, 0.0),
      3);
    update_and_check_tracking_state(
      robot_info,
      eleventh_state,
      TrackingState::OnLane,
      4);

    // Twelth state, on waypoint
    RobotState twelth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {0.0, 10.0 - 0.5 + 1e-3}, 0.0),
      3);
    update_and_check_tracking_state(
      robot_info,
      twelth_state,
      TrackingState::OnWaypoint,
      3);

    // Thirteenth state, on waypoint
    RobotState thirteenth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      request.task_id(),
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {0.0, 10.0}, 0.0),
      3);
    update_and_check_tracking_state(
      robot_info,
      thirteenth_state,
      TrackingState::OnWaypoint,
      3);

    // Forteenth state, task done, on waypoint
    RobotState forteenth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      std::nullopt,
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {0.0, 10.0}, 0.0),
      0);
    update_and_check_tracking_state(
      robot_info,
      forteenth_state,
      TrackingState::OnWaypoint,
      3);

    // Fifteenth state, task done, lost
    RobotState fifteenth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      std::nullopt, 
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {-0.5 - 1e-3, 10.0 + 0.5 + 1e-3}, 0.0),
      0);
    update_and_check_tracking_state(
      robot_info,
      fifteenth_state,
      TrackingState::Lost,
      0);

    // Sixteenth state, task done, back on waypoint
    RobotState sixteenth_state(
      std::chrono::steady_clock::now(),
      initial_state.name(),
      initial_state.model(),
      std::nullopt, 
      initial_state.mode(),
      initial_state.battery_percent(),
      Location(test_map_name, {-0.5 + 1e-3, 10.0}, 0.0),
      0);
    update_and_check_tracking_state(
      robot_info,
      sixteenth_state,
      TrackingState::OnWaypoint,
      3);
  }
}
