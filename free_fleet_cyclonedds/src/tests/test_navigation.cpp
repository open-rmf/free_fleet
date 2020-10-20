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

#include <memory>
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

  double x = 1.65683;
  double y = 0.548278;
  double yaw = -1.13961;

  auto manager =
    free_fleet::cyclonedds::CycloneDDSMiddleware::make_manager(
      dds_domain, fleet_name);
  if (!manager)
  {
    std::cerr << "[ERROR]: Failed to initialize a manager.\n";
    return 1;
  }

  free_fleet::messages::Location loc {
    0,
    0,
    x,
    y,
    yaw,
    level_name};
  free_fleet::messages::NavigationRequest request;
  request.robot_name = robot_name;
  request.task_id = task_id;
  request.path.push_back(loc);

  bool manager_loop = true;
  while (manager_loop)
  {
    manager->send_navigation_request(request);

    auto states = manager->read_states();
    if (!states.empty())
    {
      for (const auto& s : states)
      {
        if (s->task_id == task_id)
          manager_loop = false;
      }
    }
    dds_sleepfor(DDS_MSECS(100));
  }

  return 0;
}
