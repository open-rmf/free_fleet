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

#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

SCENARIO("Verify that the middleware implementation works")
{
  using namespace free_fleet::cyclonedds;
  int dds_domain = 42;
  std::string fleet_name = "test_fleet";

  GIVEN("Single server and client")
  {
    auto server = CycloneDDSMiddleware::make_server(dds_domain, fleet_name);
    REQUIRE(server);

    auto client = CycloneDDSMiddleware::make_client(dds_domain, fleet_name);
    REQUIRE(client);
  }
}
