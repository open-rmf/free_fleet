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

#include <free_fleet/agv/RobotInfo.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include "src/agv/internal_RobotInfo.hpp"

#include "src/RequestInfo.hpp"

void update_and_check_tracking_state(
  const std::shared_ptr<free_fleet::agv::RobotInfo>& robot_info,
  const free_fleet::messages::RobotState& new_state,
  free_fleet::agv::RobotInfo::TrackingState desired_tracking_state,
  std::size_t desired_tracking_index)
{
  rmf_traffic::Time update_time = std::chrono::steady_clock::now();

  free_fleet::agv::RobotInfo::Implementation::get(*robot_info).update_state(
    new_state, update_time);
  CHECK(robot_info->state() == new_state);
  CHECK(robot_info->last_updated() == update_time);

  auto tracking_estimation = robot_info->tracking_estimation();
  CHECK(tracking_estimation.first == desired_tracking_state);
  
  if (desired_tracking_state != free_fleet::agv::RobotInfo::TrackingState::Lost)
  {
    CHECK(tracking_estimation.second == desired_tracking_index);
  }
}

SCENARIO("Tests RobotInfo API")
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
    free_fleet::agv::RobotInfo::Implementation::make(
      initial_state,
      graph,
      initial_time);
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
    free_fleet::agv::RobotInfo::Implementation::get(*robot_info).update_state(
      next_state, next_time);

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
    free_fleet::agv::RobotInfo::Implementation::get(*robot_info).update_state(
      next_state, next_time);

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
    free_fleet::agv::RobotInfo::Implementation::get(*robot_info).update_state(
      next_state, next_time);

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
    free_fleet::agv::RobotInfo::Implementation::get(*robot_info).update_state(
      next_state, next_time);

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
    free_fleet::agv::RobotInfo::Implementation::get(*robot_info).update_state(
      next_state, next_time);

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
    free_fleet::agv::RobotInfo::Implementation::get(*robot_info).update_state(
      second_state, second_time);

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
    free_fleet::agv::RobotInfo::Implementation::get(*robot_info).update_state(
      third_state, third_time);

    CHECK(robot_info->state() == third_state);
    CHECK(robot_info->last_updated() == third_time);

    auto third_tracking_estimation = robot_info->tracking_estimation();
    CHECK(third_tracking_estimation.first == TrackingState::OnWaypoint);
    CHECK(third_tracking_estimation.second == 1);
  }

  GIVEN("Allocated pause request")
  {
    using PauseRequest = free_fleet::messages::PauseRequest;
    using BaseRequestInfo = free_fleet::requests::BaseRequestInfo;

    PauseRequest pause_request {
      initial_state.name,
      initial_state.task_id + 1
    };

    bool request_sent = false;
    std::shared_ptr<free_fleet::requests::RequestInfo<PauseRequest>>
      new_pause_request_info =
        std::make_shared<free_fleet::requests::RequestInfo<PauseRequest>>(
          BaseRequestInfo::RequestType::PauseRequest,
          pause_request,
          [&](const PauseRequest&) {request_sent = true;},
          std::chrono::steady_clock::now());
    REQUIRE(new_pause_request_info);

    CHECK_NOTHROW(
      free_fleet::agv::RobotInfo::Implementation::get(
        *robot_info).allocate_task(
          std::dynamic_pointer_cast<BaseRequestInfo>(new_pause_request_info)));
  }

  GIVEN("Allocated resume request")
  {
    using ResumeRequest = free_fleet::messages::ResumeRequest;
    using BaseRequestInfo = free_fleet::requests::BaseRequestInfo;

    ResumeRequest resume_request {
      initial_state.name,
      initial_state.task_id + 1
    };

    bool request_sent = false;
    std::shared_ptr<free_fleet::requests::RequestInfo<ResumeRequest>>
      new_resume_request_info =
        std::make_shared<free_fleet::requests::RequestInfo<ResumeRequest>>(
          BaseRequestInfo::RequestType::ResumeRequest,
          resume_request,
          [&](const ResumeRequest&) {request_sent = true;},
          std::chrono::steady_clock::now());
    REQUIRE(new_resume_request_info);

    CHECK_NOTHROW(
      free_fleet::agv::RobotInfo::Implementation::get(
        *robot_info).allocate_task(
          std::dynamic_pointer_cast<BaseRequestInfo>(new_resume_request_info)));
  }

  GIVEN("Allocated dock request")
  {
    using DockRequest = free_fleet::messages::DockRequest;
    using BaseRequestInfo = free_fleet::requests::BaseRequestInfo;

    DockRequest dock_request {
      initial_state.name,
      initial_state.task_id + 1,
      "mock_dock"
    };

    bool request_sent = false;
    std::shared_ptr<free_fleet::requests::RequestInfo<DockRequest>>
      new_dock_request_info =
        std::make_shared<free_fleet::requests::RequestInfo<DockRequest>>(
          BaseRequestInfo::RequestType::DockRequest,
          dock_request,
          [&](const DockRequest&) {request_sent = true;},
          std::chrono::steady_clock::now());
    REQUIRE(new_dock_request_info);

    CHECK_NOTHROW(
      free_fleet::agv::RobotInfo::Implementation::get(
        *robot_info).allocate_task(
          std::dynamic_pointer_cast<BaseRequestInfo>(new_dock_request_info)));
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

    using RelocalizationRequest = free_fleet::messages::RelocalizationRequest;
    using BaseRequestInfo = free_fleet::requests::BaseRequestInfo;
    RelocalizationRequest relocalization_request {
      initial_state.name,
      initial_state.task_id + 1,
      reloc_loc,
      3
    };

    bool request_sent = false;
    std::shared_ptr<free_fleet::requests::RequestInfo<RelocalizationRequest>>
      new_reloc_request_info =
        std::make_shared<free_fleet::requests::RequestInfo<RelocalizationRequest>>(
          BaseRequestInfo::RequestType::RelocalizationRequest,
          relocalization_request,
          [&](const RelocalizationRequest&) {request_sent = true;},
          std::chrono::steady_clock::now());
    REQUIRE(new_reloc_request_info);

    CHECK_NOTHROW(
      free_fleet::agv::RobotInfo::Implementation::get(
        *robot_info).allocate_task(
          std::dynamic_pointer_cast<BaseRequestInfo>(new_reloc_request_info)));
  }

  GIVEN("Allocated navigation request")
  {
    using NavigationRequest = free_fleet::messages::NavigationRequest;
    using BaseRequestInfo = free_fleet::requests::BaseRequestInfo;

    NavigationRequest nav_request {
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
    std::shared_ptr<free_fleet::requests::RequestInfo<NavigationRequest>>
      new_nav_request_info =
        std::make_shared<free_fleet::requests::RequestInfo<NavigationRequest>>(
          BaseRequestInfo::RequestType::NavigationRequest,
          nav_request,
          [&](const NavigationRequest&) {request_sent = true;},
          std::chrono::steady_clock::now());
    REQUIRE(new_nav_request_info);

    CHECK_NOTHROW(
      free_fleet::agv::RobotInfo::Implementation::get(
        *robot_info).allocate_task(
          std::dynamic_pointer_cast<BaseRequestInfo>(new_nav_request_info)));
  }

  GIVEN("Allocate navigation request, update multiple states, throughout the path")
  {
    using NavigationRequest = free_fleet::messages::NavigationRequest;
    using BaseRequestInfo = free_fleet::requests::BaseRequestInfo;

    uint32_t new_task_id = initial_state.task_id + 1;
    NavigationRequest nav_request {
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
    second_wp.location.y = 0.0;
    nav_request.path.push_back(second_wp);

    auto third_wp = wp;
    third_wp.index = 0;
    third_wp.location.x = 0.0;
    third_wp.location.y = 0.0;
    nav_request.path.push_back(third_wp);

    auto forth_wp = wp;
    forth_wp.index = 3;
    forth_wp.location.x = 0.0;
    forth_wp.location.y = 10.0;
    nav_request.path.push_back(forth_wp);

    bool request_sent = false;
    std::shared_ptr<free_fleet::requests::RequestInfo<NavigationRequest>>
      new_nav_request_info =
        std::make_shared<free_fleet::requests::RequestInfo<NavigationRequest>>(
          BaseRequestInfo::RequestType::NavigationRequest,
          nav_request,
          [&](const NavigationRequest&) {request_sent = true;},
          std::chrono::steady_clock::now());
    REQUIRE(new_nav_request_info);

    CHECK_NOTHROW(
      free_fleet::agv::RobotInfo::Implementation::get(
        *robot_info).allocate_task(
          std::dynamic_pointer_cast<BaseRequestInfo>(new_nav_request_info)));

    // Second state, starting on the path
    auto second_state = initial_state;
    second_state.task_id = new_task_id;
    second_state.location.x = 0.0;
    second_state.location.y = 0.0;
    update_and_check_tracking_state(
      robot_info,
      second_state,
      TrackingState::OnWaypoint,
      0);

    // Third state, on the lane
    auto third_state = initial_state;
    third_state.task_id = new_task_id;
    third_state.location.x = 0.0 + 0.5 + 1e-3;
    third_state.location.y = 0.0;
    third_state.path_target_index = 1;
    update_and_check_tracking_state(
      robot_info,
      third_state,
      TrackingState::OnLane,
      0);

    // Forth state, on the lane
    auto forth_state = initial_state;
    forth_state.task_id = new_task_id;
    forth_state.location.x = 10.0 - 0.5 - 1e-3;
    forth_state.location.y = 0.0;
    forth_state.path_target_index = 1;
    update_and_check_tracking_state(
      robot_info,
      forth_state,
      TrackingState::OnLane,
      0);

    // Fifth state, on waypoint
    auto fifth_state = initial_state;
    fifth_state.task_id = new_task_id;
    fifth_state.location.x = 10.0 - 0.5 + 1e-3;
    fifth_state.location.y = 1e-3;
    fifth_state.path_target_index = 1;
    update_and_check_tracking_state(
      robot_info,
      fifth_state,
      TrackingState::OnWaypoint,
      1);

    // Sixth state, on waypoint
    auto sixth_state = initial_state;
    sixth_state.task_id = new_task_id;
    sixth_state.location.x = 10.0;
    sixth_state.location.y = 0.5 - 1e-3;
    sixth_state.path_target_index = 2;
    update_and_check_tracking_state(
      robot_info,
      sixth_state,
      TrackingState::OnWaypoint,
      1);

    // Seventh state, on lane, a bit lost near waypoint, but with enough
    // information to get the lane
    auto seventh_state = initial_state;
    seventh_state.task_id = new_task_id;
    seventh_state.location.x = 10.0;
    seventh_state.location.y = 0.5 + 1e-3;
    seventh_state.path_target_index = 2;
    update_and_check_tracking_state(
      robot_info,
      seventh_state,
      TrackingState::OnLane,
      1);
    
    // Eigth state, on lane, on previous waypoint
    auto eighth_state = initial_state;
    eighth_state.task_id = new_task_id;
    eighth_state.location.x = 10.0;
    eighth_state.location.y = 0.5 - 1e-3;
    eighth_state.path_target_index = 2;
    update_and_check_tracking_state(
      robot_info,
      eighth_state,
      TrackingState::OnWaypoint,
      1);

    // Ninth state, on lane
    auto ninth_state = initial_state;
    ninth_state.task_id = new_task_id;
    ninth_state.location.x = 10.0 - 0.5 - 1e-3;
    ninth_state.location.y = 0.0;
    ninth_state.path_target_index = 2;
    update_and_check_tracking_state(
      robot_info,
      ninth_state,
      TrackingState::OnLane,
      1);

    // Tenth state, on waypoint
    auto tenth_state = initial_state;
    tenth_state.task_id = new_task_id;
    tenth_state.location.x = 0.5 - 1e-3;
    tenth_state.location.y = 0.0;
    tenth_state.path_target_index = 3;
    update_and_check_tracking_state(
      robot_info,
      tenth_state,
      TrackingState::OnWaypoint,
      0);
    
    // Eleventh state, on lane
    auto eleventh_state = initial_state;
    eleventh_state.task_id = new_task_id;
    eleventh_state.location.x = 0.0;
    eleventh_state.location.y = 0.5 + 1e-3;
    eleventh_state.path_target_index = 3;
    update_and_check_tracking_state(
      robot_info,
      eleventh_state,
      TrackingState::OnLane,
      4);

    // Twelth state, on waypoint
    auto twelth_state = initial_state;
    twelth_state.task_id = new_task_id;
    twelth_state.location.x = 0.0;
    twelth_state.location.y = 10.0 - 0.5 + 1e-3;
    twelth_state.path_target_index = 3;
    update_and_check_tracking_state(
      robot_info,
      twelth_state,
      TrackingState::OnWaypoint,
      3);

    // Thirteenth state, on waypoint
    auto thirteenth_state = initial_state;
    thirteenth_state.task_id = new_task_id;
    thirteenth_state.location.x = 0.0;
    thirteenth_state.location.y = 10.0;
    thirteenth_state.path_target_index = 3;
    update_and_check_tracking_state(
      robot_info,
      thirteenth_state,
      TrackingState::OnWaypoint,
      3);

    // Forteenth state, task done, on waypoint
    auto forteenth_state = initial_state;
    forteenth_state.task_id = 0;
    forteenth_state.location.x = 0.0;
    forteenth_state.location.y = 10.0;
    forteenth_state.path_target_index = 0;
    update_and_check_tracking_state(
      robot_info,
      forteenth_state,
      TrackingState::OnWaypoint,
      3);

    // Fifteenth state, task done, lost
    auto fifteenth_state = initial_state;
    fifteenth_state.task_id = 0;
    fifteenth_state.location.x = -0.5 - 1e-3;
    fifteenth_state.location.y = 10.0 + 0.5 + 1e-3;
    fifteenth_state.path_target_index = 0;
    update_and_check_tracking_state(
      robot_info,
      fifteenth_state,
      TrackingState::Lost,
      0);

    // Sixteenth state, task done, back on waypoint
    auto sixteenth_state = initial_state;
    sixteenth_state.task_id = 0;
    sixteenth_state.location.x = -0.5 + 1e-3;
    sixteenth_state.location.y = 10.0;
    sixteenth_state.path_target_index = 0;
    update_and_check_tracking_state(
      robot_info,
      sixteenth_state,
      TrackingState::OnWaypoint,
      3);
  }
}
