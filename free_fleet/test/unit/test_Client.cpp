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

#include <rmf_utils/catch.hpp>

#include <iostream>
#include <free_fleet/agv/Client.hpp>

#include "mock_Middleware.hpp"
#include "mock_StatusHandle.hpp"
#include "mock_CommandHandle.hpp"
#include "src/agv/internal_Client.hpp"

SCENARIO("Verify that a client can run")
{
  const std::string robot_name = "mock_robot";
  const std::string robot_model = "mock_robot_model";
  auto ch = std::make_shared<free_fleet::MockCommandHandle>();
  auto sh = std::make_shared<free_fleet::MockStatusHandle>();
  auto m = std::make_shared<free_fleet::MockMiddleware>();

  GIVEN("All valid")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      robot_model,
      ch,
      sh,
      m);
    REQUIRE(client);
    CHECK(!client->started());
  }

  GIVEN("Empty robot name")
  {
    auto client = free_fleet::agv::Client::make(
      "",
      robot_model,
      ch,
      sh,
      m);
    CHECK(!client);
  }

  GIVEN("Empty robot model")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      "",
      ch,
      sh,
      m);
    CHECK(!client);
  }

  GIVEN("Invalid CommandHandle")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      robot_model,
      nullptr,
      sh,
      m);
    CHECK(!client);
  }

  GIVEN("Invalid StatusHandle")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      robot_model,
      ch,
      nullptr,
      m);
    CHECK(!client);
  }
  GIVEN("Invalid Middleware")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      robot_model,
      ch,
      sh,
      nullptr);
    CHECK(!client);
  }

  GIVEN("Starting with bad frequency")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      robot_model,
      ch,
      sh,
      m);
    REQUIRE(client);
    CHECK(!client->started());
    CHECK_THROWS(client->run(0));
    CHECK_THROWS(client->start_async(0));
  }

  GIVEN("Running once")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      robot_model,
      ch,
      sh,
      m);
    REQUIRE(client);
    auto& impl = free_fleet::agv::Client::Implementation::get(*client);
    CHECK_NOTHROW(impl.run_once());
    CHECK(!client->started());
  }
}
