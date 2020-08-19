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

#ifndef SRC__PUBLISHER_HPP
#define SRC__PUBLISHER_HPP

#include <memory>

#include <dds/dds.h>

namespace free_fleet {
namespace cyclonedds {

template <typename Message>
class Publisher
{
public:

  Publisher(
      const dds_entity_t& participant,
      const dds_topic_descriptor_t* topic_desc,
      const std::string& topic_name)
  : _ready(false)
  {
    _topic = dds_create_topic(
        participant, topic_desc, topic_name.c_str(), NULL, NULL);
    if (_topic < 0)
    {
      DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-_topic));
      return;
    }

    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    _writer = dds_create_writer(participant, _topic, qos, NULL);
    if (_writer < 0)
    {
      DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-_writer));
      return;
    }
    dds_delete_qos(qos);
    _ready = true;
  }

  ~Publisher()
  {}

  bool is_ready()
  {
    return _ready;
  }

  bool write(Message* msg)
  {
    dds_return_t return_code = dds_write(_writer, msg);
    if (return_code != DDS_RETCODE_OK)
    {
      DDS_FATAL("dds_write failed: %s", dds_strretcode(-return_code));
      return false;
    }
    return true;
  }

private:

  dds_entity_t _topic;
  dds_entity_t _writer;
  bool _ready;
};

} // namespace cyclonedds
} // namespace free_fleet

#endif // SRC__PUBLISHER_HPP
