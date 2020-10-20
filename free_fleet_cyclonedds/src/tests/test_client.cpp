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

#include<iostream>

#include <dds/dds.h>

#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Please request using the following format," << std::endl;
    std::cout << "<Executable> <DDS Domain ID> <Fleet name> <Robot name>"
      << std::endl;
    return 1;
  }

  int dds_domain = strtod(argv[1], NULL);
  std::string fleet_name(argv[2]);
  std::string robot_name(argv[3]);

  auto client =
    free_fleet::cyclonedds::CycloneDDSMiddleware::make_client(
      dds_domain, fleet_name);
  if (!client)
  {
    std::cerr << "[ERROR]: Failed to initialize a client.\n";
    return 1;
  }

  int count = 0;

  while (true)
  {
    if (count >= 10)
    {
      free_fleet::messages::RobotState state;
      state.name = robot_name;
      state.model = "test_model";
      state.task_id = "test_id";
      state.mode.mode = free_fleet::messages::RobotMode::MODE_MOVING;
      state.battery_percent = 20.3;
      state.location = free_fleet::messages::Location{
        32,
        24000,
        0.0,
        10.0,
        20.0,
        "test_level"};
      std::cout << "[CLIENT]: Publishing a message.\n";
      client->send_state(state);
      count = 0;
    }
    
    auto request = client->read_navigation_request();
    if (request && request->robot_name == robot_name)
    {
      std::cout << "[CLIENT]: Received a request of id: "
        << request->task_id << std::endl;
    }

    dds_sleepfor(DDS_MSECS(50));
    count++;
  }

  std::cout << "All done" << std::endl;
  return 0;
}

