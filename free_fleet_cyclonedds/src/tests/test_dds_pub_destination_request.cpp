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

#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include <iostream>

#include <dds/dds.h>

#include "../messages/FleetMessages.h"
#include "../dds_utils/common.hpp"

int main (int argc, char ** argv)
{
  if (argc < 5)
  {
    std::cout << "Please request using the following format," << std::endl;
    std::cout << "<Executable> <Fleet name> <Robot Name> <Task ID> <X> <Y> <Yaw>" << std::endl;
    return 1;
  }

  std::string fleet_name(argv[1]);
  std::string robot_name(argv[2]);
  std::string task_id(argv[3]);
  double x = strtod(argv[4], NULL);
  double y = strtod(argv[5], NULL);
  double yaw = strtod(argv[6], NULL);
  std::string level_name = "B1";

  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  dds_qos_t *qos;
  FreeFleetData_DestinationRequest* msg;
  msg = FreeFleetData_DestinationRequest__alloc();
  uint32_t status = 0;
  (void)argc;
  (void)argv;

  /* Create a Participant. */
  participant = dds_create_participant(42, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &FreeFleetData_DestinationRequest_desc, "destination_request", 
    NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a Writer. */
  qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  writer = dds_create_writer (participant, topic, qos, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_write: %s\n", dds_strretcode(-writer));
  dds_delete_qos(qos);

  printf("=== [Publisher]  Waiting for a reader to be discovered ...\n");
  fflush (stdout);

  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

  while(!(status & DDS_PUBLICATION_MATCHED_STATUS))
  {
    rc = dds_get_status_changes (writer, &status);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

    /* Polling sleep. */
    dds_sleepfor (DDS_MSECS (20));
  }

  /* Create a message to write. */
  msg->fleet_name = free_fleet::common::dds_string_alloc_and_copy(fleet_name);
  msg->robot_name = free_fleet::common::dds_string_alloc_and_copy(robot_name);
  msg->task_id = free_fleet::common::dds_string_alloc_and_copy(task_id);

  // time is weird for now
  msg->destination.sec = 123;
  msg->destination.nanosec = 123;
  msg->destination.x = x;
  msg->destination.y = y;
  msg->destination.yaw = yaw;
  msg->destination.level_name = free_fleet::common::dds_string_alloc_and_copy(level_name);

  printf ("=== [Publisher]  Writing : ");
  printf ("Message: level_name %s\n", msg->destination.level_name);
  fflush (stdout);

  rc = dds_write (writer, msg);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  FreeFleetData_DestinationRequest_free(msg, DDS_FREE_ALL);
  return EXIT_SUCCESS;
}
