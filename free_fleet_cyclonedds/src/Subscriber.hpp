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

#ifndef SRC__SUBSCRIBER_HPP
#define SRC__SUBSCRIBER_HPP

#include <memory>
#include <vector>

#include <dds/dds.h>

namespace free_fleet {
namespace cyclonedds {

template <typename Message, size_t MaxSamplesNum = 1>
class Subscriber
{
public:

  Subscriber(
      const dds_entity_t& participant, 
      const dds_topic_descriptor_t* topic_desc, 
      const std::string& topic_name)
  : _ready(false)
  {
    _topic = dds_create_topic(
        participant, topic_desc, topic_name.c_str(), NULL, NULL);
    if (_topic < 0)
    {
      DDS_FATAL(
          "dds_create_topic: %s\n", dds_strretcode(-_topic));
      return;
    }

    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    _reader = dds_create_reader(participant, _topic, qos, NULL);
    if (_reader < 0)
    {
      DDS_FATAL(
          "dds_create_reader: %s\n", dds_strretcode(-_reader));
      return;
    }
    dds_delete_qos(qos);

    for (std::size_t i = 0; i < _shared_msgs.size(); ++i)
    {
      _shared_msgs[i] =
          std::shared_ptr<Message>((Message*)dds_alloc(sizeof(Message)));
      _samples[i] = (void*)_shared_msgs[i].get();
    }

    _ready = true;
  }

  ~Subscriber()
  {}

  bool is_ready()
  {
    return _ready;
  }

  std::vector<std::shared_ptr<const Message>> read()
  {
    std::vector<std::shared_ptr<const Message>> msgs;
    if (!is_ready())
      return msgs;

    dds_return_t return_code =
        dds_take(_reader, _samples, _infos, MaxSamplesNum, MaxSamplesNum);
    if (return_code < 0)
    {
      DDS_FATAL("dds_take: %s\n", dds_strretcode(-return_code));
      msgs.clear();
      return msgs;
    }
    
    if (return_code > 0)
    {
      for (std::size_t i = 0; i < MaxSamplesNum; ++i)
      {
        if (_infos[i].valid_data)
          msgs.push_back(std::shared_ptr<const Message>(_shared_msgs[i]));
      }
      return msgs;
    }
    msgs.clear();
    return msgs;
  }

private:

  dds_entity_t _topic;
  dds_entity_t _reader;
  
  std::array<std::shared_ptr<Message>, MaxSamplesNum> _shared_msgs;
  void* _samples[MaxSamplesNum];
  dds_sample_info_t _infos[MaxSamplesNum];

  bool _ready;
};

} // namespace cyclonedds
} // namespace free_fleet


#endif // SRC__SUBSCRIBER_HPP
