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

#include "ros/ros.h"
#include "dds/dds.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "free_fleet_client");
  ROS_INFO("greetings from free_fleet_client");

  // set up a DDS writer
  dds_entity_t topic, writer;
  dds_return_t rc;

  // todo: pull dds domain from a config file
  dds_entity_t participant = dds_create_participant(0, NULL, NULL);
  if (participant < 0)
  {
    ROS_FATAL("couldn't create DDS participant: %s", dds_strretcode(-participant));
    return 1;
  }

  rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK)
  {
    ROS_FATAL("dds_delete: %s", dds_strretcode(-rc));
  }

  return 0;
}
