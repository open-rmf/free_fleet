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

#include <atomic>
#include <thread>

#include <free_fleet/Worker.hpp>
#include <free_fleet/Executor.hpp>

#include <rmf_utils/catch.hpp>

#include "mock_Worker.hpp"

SCENARIO("Test Executor")
{
  std::atomic<bool> ran = false; 
  std::unique_ptr<free_fleet::Worker> worker(new free_fleet::MockWorker(
    123,
    [&]()
  {
    ran = true;
  }));
  REQUIRE(worker != nullptr);

  free_fleet::Executor executor(std::move(worker));

  GIVEN("Asynchronous starting")
  {
    REQUIRE(!executor.started());
    executor.start_async(std::chrono::nanoseconds(1000));
    CHECK(executor.started());
  }

  GIVEN("Asynchronous stopping")
  {
    REQUIRE(!executor.started());
    executor.start_async(std::chrono::nanoseconds(1000));
    REQUIRE(executor.started());
    executor.stop();
    CHECK(!executor.started());
  }

  GIVEN("Asynchronous started running")
  {
    REQUIRE(!executor.started());
    executor.start_async(std::chrono::nanoseconds(1));
    REQUIRE(executor.started());
    std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    executor.stop();
    REQUIRE(!executor.started());
    CHECK(ran.load());
  }

  GIVEN("Worker ID")
  {
    free_fleet::Worker* w_ptr = executor.worker();
    REQUIRE(w_ptr != nullptr);
    auto* mw_ptr = static_cast<free_fleet::MockWorker*>(w_ptr);
    REQUIRE(mw_ptr != nullptr);
    CHECK(mw_ptr->id() == 123);
    mw_ptr->id(321);
    CHECK(mw_ptr->id() == 321);
  }
}
