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

#include <dds/dds.h>

#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include <free_fleet/messages/NavigationRequest.hpp>

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cout << "Please request using the following format," << std::endl;
    std::cout << "<Executable> <DDS Domain ID> <Fleet name>" << std::endl;
    return 1;
  }

  int dds_domain = strtod(argv[1], NULL);
  std::string fleet_name(argv[2]);

  auto manager =
    free_fleet::cyclonedds::CycloneDDSMiddleware::make_manager(
      dds_domain, fleet_name);
  if (!manager)
  {
    std::cerr << "[ERROR]: Failed to initialize a manager.\n";
    return 1;
  }

  while (true)
  {
    auto states = manager->read_states();
    if (!states.empty())
    {
      for (const auto& s : states)
      {
        std::cout << "Got a state from client: "<< s->name
          << ", task: " << s->task_id
          << ", battery: " << s->battery_percent
          << ", mode: " << s->mode.mode
          << ", [" << s->location.level_name
          << " " << s->location.x
          << " " << s->location.y
          << " " << s->location.yaw << "]"
          << ", path length: " << s->path.size() << "\n";
      }
    }
    dds_sleepfor(DDS_MSECS(100));
  }

  std::cout << "All done" << std::endl;
  return 0;
}


