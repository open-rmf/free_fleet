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

#include <ros/ros.h>
#include <dds/dds.h>

#include "../free_fleet/FreeFleet.h"


// Written using the hello world example from Cyclonedds

int main(int argc, char** argv)
{
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  dds_qos_t* qos;

  FreeFleetData_Location msg;
  uint32_t status = 0;

  /* Create a Participant. */
  participant = dds_create_participant (DDS_DOMAIN_DEFAULT, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &FreeFleetData_Location_desc, "fake_fleet/location_command", 
    NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a best effort Writer (UDP) */
  qos = dds_create_qos ();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  writer = dds_create_writer (participant, topic, qos, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_write: %s\n", dds_strretcode(-writer));
  dds_delete_qos(qos);

  /* Create a message to write. */
  msg.sec = 123;
  msg.nanosec = 123;
  msg.x = 123.123;
  msg.y = 123.123;
  msg.yaw = 123.123;
  msg.level_name = "B1";

  std::cout << "=== [Publisher]  Writing : " << std::endl;
  std::cout << "Message: " << std::endl;
  std::cout << msg.sec << " " << msg.nanosec << std::endl;
  std::cout << msg.x << " " << msg.y << " " << msg.yaw << std::endl;
  std::cout << "level name: " << msg.level_name << std::endl;

  rc = dds_write (writer, &msg);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  return EXIT_SUCCESS;
}
