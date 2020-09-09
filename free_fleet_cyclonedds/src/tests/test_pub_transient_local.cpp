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

#include <memory>
#include <iostream>

#include <dds/dds.h>

#include "../messages/MiddlewareMessages.h"
#include "../messages/utils.hpp"
#include "../Publisher.hpp"

int main()
{
  /* Create a participant */
  dds_entity_t participant =
      dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return 1;
  }

  auto pub = free_fleet::cyclonedds::Publisher<MiddlewareMessages_Location>::make(
      participant,
      &MiddlewareMessages_Location_desc,
      "test_pubsub_transient_local",
      true);
  if (!pub)
  {
    std::cout << "Failed to create publisher" << std::endl;
    return 1;
  }

  MiddlewareMessages_Location* msg = MiddlewareMessages_Location__alloc();
  for (int i = 0; i < 3; ++i)
  {
    if (!pub->write(msg))
    {
      std::cout << "Failed to publish message" << std::endl;
      return 1;
    }
    std::cout << "Published a transient_local message" << std::endl;
    dds_sleepfor(DDS_MSECS(500));
  }

  while (true)
  {
    std::cout << "Staying alive..." << std::endl;
    dds_sleepfor(DDS_MSECS(1000));
  }

  MiddlewareMessages_Location_free(msg, DDS_FREE_ALL);
  dds_return_t rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  std::cout << "Published a message." << std::endl;
  return 0;
}
