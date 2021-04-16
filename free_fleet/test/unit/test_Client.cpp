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

#include "mock_StatusHandle.hpp"
#include "mock_CommandHandle.hpp"
#include "mock_ClientMiddleware.hpp"
#include "src/agv/internal_Client.hpp"

SCENARIO("Verify that a client can run")
{
  const std::string robot_name = "mock_robot";
  const std::string robot_model = "mock_robot_model";
  auto ch = std::make_shared<free_fleet::MockCommandHandle>();
  auto sh = std::make_shared<free_fleet::MockStatusHandle>();
  std::unique_ptr<free_fleet::transport::ClientMiddleware> m(
    new free_fleet::MockClientMiddleware());


  GIVEN("All valid")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      robot_model,
      ch,
      sh,
      std::move(m));
    REQUIRE(client);
    CHECK(!client->started());
  }

  GIVEN("Starting with bad frequency")
  {
    auto client = free_fleet::agv::Client::make(
      robot_name,
      robot_model,
      ch,
      sh,
      std::move(m));
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
      std::move(m));
    REQUIRE(client);
    auto& impl = free_fleet::agv::Client::Implementation::get(*client);
    CHECK_NOTHROW(impl.run_once());
    CHECK(!client->started());
  }
}

class MockClientMiddlewareWithServer : public free_fleet::MockClientMiddleware
{
public:

  MockClientMiddlewareWithServer()
  {}

  void received_dock_request(
    const std::string& robot_name,
    uint32_t task_id, 
    const std::string& dock_name)
  {
    if (dock_request_callback)
    {
      free_fleet::messages::DockRequest request {
        robot_name, task_id, dock_name};
      dock_request_callback(request);
    }
  }

  void received_pause_request(const std::string& robot_name, uint32_t task_id)
  {
    if (pause_request_callback)
    {
      free_fleet::messages::PauseRequest request {robot_name, task_id};
      pause_request_callback(request);
    }
  }
  void received_resume_request(const std::string& robot_name, uint32_t task_id)
  {
    if (resume_request_callback)
    {
      free_fleet::messages::ResumeRequest request {robot_name, task_id};
      resume_request_callback(request);
    }
  }

  void received_navigation_request(
    const std::string& robot_name, uint32_t task_id)
  {
    if (navigation_request_callback)
    {
      free_fleet::messages::NavigationRequest request;
      request.robot_name = robot_name;
      request.task_id = task_id;
      navigation_request_callback(request);
    }
  }

  void received_relocalization_request(
    const std::string& robot_name, uint32_t task_id)
  {
    if (relocalization_request_callback)
    {
      free_fleet::messages::RelocalizationRequest request;
      request.robot_name = robot_name;
      request.task_id = task_id;
      relocalization_request_callback(request);
    }
  }
};

SCENARIO("Testing receiving requests")
{
  const std::string robot_name = "mock_robot";
  const std::string robot_model = "mock_robot_model";
  auto ch = std::make_shared<free_fleet::MockCommandHandle>();
  auto sh = std::make_shared<free_fleet::MockStatusHandle>();
  std::unique_ptr<free_fleet::transport::ClientMiddleware> m(
    new MockClientMiddlewareWithServer());
  auto client = free_fleet::agv::Client::make(
    robot_name,
    robot_model,
    ch,
    sh,
    std::move(m));
  REQUIRE(client);

  auto& impl = free_fleet::agv::Client::Implementation::get(*client);
  impl.set_callbacks();
  MockClientMiddlewareWithServer* middleware =
    dynamic_cast<MockClientMiddlewareWithServer*>(impl.middleware.get());
  REQUIRE(impl.middleware);

  GIVEN("Receiving another robot's dock request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_dock_request("wrong_robot", 1, "mock_dock");
    CHECK(impl.task_id == 0);
    CHECK(impl.task_ids.find(1) == impl.task_ids.end());
  }

  GIVEN("Receiving another robot's pause request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_pause_request("wrong_robot", 1);
    CHECK(impl.task_id == 0);
    CHECK(impl.task_ids.find(1) == impl.task_ids.end());
  }
  GIVEN("Receiving another robot's resume request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_resume_request("wrong_robot", 1);
    CHECK(impl.task_id == 0);
    CHECK(impl.task_ids.find(1) == impl.task_ids.end());
  }

  GIVEN("Receiving another robot's navigation request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_navigation_request("wrong_robot", 1);
    CHECK(impl.task_id == 0);
    CHECK(impl.task_ids.find(1) == impl.task_ids.end());
  }

  GIVEN("Receiving another robot's relocalization request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_relocalization_request("wrong_robot", 1);
    CHECK(impl.task_id == 0);
    CHECK(impl.task_ids.find(1) == impl.task_ids.end());
  }

  GIVEN("Receiving dock request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_dock_request(robot_name, 1, "mock_dock");
    CHECK(impl.task_id == 1);
    CHECK(impl.task_ids.find(1) != impl.task_ids.end());
  }

  GIVEN("Receiving pause request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_pause_request(robot_name, 1);
    CHECK(impl.task_id == 1);
    CHECK(impl.task_ids.find(1) != impl.task_ids.end());
  }

  GIVEN("Receiving resume request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_resume_request(robot_name, 1);
    CHECK(impl.task_id == 1);
    CHECK(impl.task_ids.find(1) != impl.task_ids.end());
  }

  GIVEN("Receiving navigation request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_navigation_request(robot_name, 1);
    CHECK(impl.task_id == 1);
    CHECK(impl.task_ids.find(1) != impl.task_ids.end());
  }

  GIVEN("Receiving relocalization request")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_relocalization_request(robot_name, 1);
    CHECK(impl.task_id == 1);
    CHECK(impl.task_ids.find(1) != impl.task_ids.end());
  }

  GIVEN("Receiving multiple requests")
  {
    REQUIRE(impl.task_id == 0);
    middleware->received_dock_request(robot_name, 1, "mock_dock");
    REQUIRE(impl.task_id == 1);
    CHECK(impl.task_ids.find(1) != impl.task_ids.end());
    middleware->received_pause_request(robot_name, 2);
    REQUIRE(impl.task_id == 2);
    CHECK(impl.task_ids.find(2) != impl.task_ids.end());
    middleware->received_resume_request(robot_name, 3);
    CHECK(impl.task_id == 3);
    CHECK(impl.task_ids.find(3) != impl.task_ids.end());
    middleware->received_navigation_request(robot_name, 4);
    CHECK(impl.task_id == 4);
    CHECK(impl.task_ids.find(4) != impl.task_ids.end());
    middleware->received_relocalization_request(robot_name, 5);
    CHECK(impl.task_id == 5);
    CHECK(impl.task_ids.find(5) != impl.task_ids.end());
  }
}
