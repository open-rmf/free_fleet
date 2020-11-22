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

#include <iostream>

#include <rmf_utils/catch.hpp>

#include <free_fleet/agv/Client.hpp>

#include "mock_Middleware.hpp"
#include "mock_StatusHandle.hpp"
#include "mock_CommandHandle.hpp"

SCENARIO("Verify that a Client can be created")
{
  auto ch = std::make_shared<free_fleet::MockCommandHandle>();
  auto sh = std::make_shared<free_fleet::MockStatusHandle>();
  auto m = std::make_shared<free_fleet::MockMiddleware>();

  auto client = free_fleet::agv::Client::make(
    "mock_robot_name",
    "mock_robot_model",
    ch,
    sh,
    m);
  REQUIRE(client);
}
