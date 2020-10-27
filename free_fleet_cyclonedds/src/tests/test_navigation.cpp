/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <dds/dds.h>

#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include <free_fleet/messages/NavigationRequest.hpp>

int main(int argc, char** argv)
{
  if (argc < 6)
  {
    std::cout << "Please request using the following format," << std::endl;
    std::cout << "<Executable> <Domain ID> <Fleet Name> <Robot Name> <Task ID> <Level Name>" << std::endl;
    return 1;
  }

  int dds_domain = strtod(argv[1], NULL);
  std::string fleet_name(argv[2]);
  std::string robot_name(argv[3]);
  std::string task_id(argv[4]);
  std::string level_name(argv[5]);

  auto server =
    free_fleet::cyclonedds::CycloneDDSMiddleware::make_server(
      dds_domain, fleet_name);
  if (!server)
  {
    std::cerr << "[ERROR]: Failed to initialize a server.\n";
    return 1;
  }

  free_fleet::messages::NavigationRequest request;
  request.robot_name = robot_name;
  request.task_id = task_id;
  request.path.push_back(
    free_fleet::messages::Waypoint{
      0,
      free_fleet::messages::Location{0, 0, -1.3461, -0.5168, 0.0, level_name}
    });
  request.path.emplace_back(
    free_fleet::messages::Waypoint{
      1,
      free_fleet::messages::Location{0, 0, 1.31282, -0.6335, 0.0, level_name}
    });
  request.path.emplace_back(
    free_fleet::messages::Waypoint{
      2,
      free_fleet::messages::Location{0, 0, 1.954647, 0.6918, 0.0, level_name}
    });
  request.path.emplace_back(
    free_fleet::messages::Waypoint{
      3,
      free_fleet::messages::Location{0, 0, -1.67124, 0.9419, 0.0, level_name}
    });
  request.path.emplace_back(
    free_fleet::messages::Waypoint{
      4,
      free_fleet::messages::Location{0, 0, -0.10419, -1.775, 0.0, level_name}
    });

  bool server_loop = true;
  int count = 0;
  while (server_loop)
  {
    server->send_navigation_request(request);
    count++;

    auto states = server->read_states();
    if (!states.empty())
    {
      for (const auto& s : states)
      {
        if (s->task_id == task_id)
          server_loop = false;
      }
    }
    dds_sleepfor(DDS_MSECS(100));
  }

  std::cout << "Sent navigation request and received feedback after "
    << count << " iterations." << std::endl;
  return 0;
}
