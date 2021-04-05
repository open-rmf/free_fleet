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

#include <free_fleet/Manager.hpp>
#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/SimpleCoordinateTransformer.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include "mock_Middleware.hpp"

SCENARIO("Test make Manager")
{
  std::string fleet_name = "test_fleet";
  std::shared_ptr<rmf_traffic::agv::Graph> graph(new rmf_traffic::agv::Graph);
  auto m = std::make_shared<free_fleet::MockMiddleware>();
  auto ct = free_fleet::SimpleCoordinateTransformer::make(
    1.0,
    0.0,
    0.0,
    0.0);
  free_fleet::Manager::TimeNow time_now_fn =
    [](){ return std::chrono::steady_clock::now(); };
  free_fleet::Manager::RobotUpdatedCallback cb =
    [](const std::shared_ptr<free_fleet::agv::RobotInfo>&){};
  
  GIVEN("All valid")
  {
    auto manager = free_fleet::Manager::make(
      fleet_name,
      graph,
      m,
      ct,
      time_now_fn,
      cb);
    CHECK(manager);
  }

  GIVEN("Empty fleet name")
  {
    auto manager = free_fleet::Manager::make(
      "",
      graph,
      m,
      ct,
      time_now_fn,
      cb);
    CHECK(!manager);
  }

  GIVEN("Invalid graph")
  {
    auto manager = free_fleet::Manager::make(
      fleet_name,
      nullptr,
      m,
      ct,
      time_now_fn,
      cb);
    CHECK(!manager);
  }

  GIVEN("Invalid Middleware")
  {
    auto manager = free_fleet::Manager::make(
      fleet_name,
      graph,
      nullptr,
      ct,
      time_now_fn,
      cb);
    CHECK(!manager);
  }

  GIVEN("Invalid CoordinateTransformer")
  {
    auto manager = free_fleet::Manager::make(
      fleet_name,
      graph,
      m,
      nullptr,
      time_now_fn,
      cb);
    CHECK(!manager);
  }

  GIVEN("Starting with bad frequency")
  {
    auto manager = free_fleet::Manager::make(
      fleet_name,
      graph,
      m,
      ct,
      time_now_fn,
      cb);
    REQUIRE(manager);
    CHECK_THROWS(manager->start(0));
  }
}
