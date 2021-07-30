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

  using SharedPtr = std::shared_ptr<Subscriber<Message, MaxSamplesNum>>;

  static SharedPtr make(
      const dds_entity_t& participant, 
      const dds_topic_descriptor_t* topic_desc, 
      const std::string& topic_name,
      bool transient_local = false)
  {
    dds_entity_t topic = dds_create_topic(
        participant, topic_desc, topic_name.c_str(), NULL, NULL);
    if (topic < 0)
    {
      DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));
      return nullptr;
    }

    dds_qos_t* qos = dds_create_qos();
    if (transient_local)
    {
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, 0);
      dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
    }
    else
    {
      dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    }

    dds_entity_t reader = dds_create_reader(participant, topic, qos, NULL);
    if (reader < 0)
    {
      DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
      return nullptr;
    }

    dds_delete_qos(qos);

    SharedPtr subscriber(new Subscriber<Message, MaxSamplesNum>());
    subscriber->_topic = std::move(topic);
    subscriber->_reader = std::move(reader);
    for (std::size_t i = 0; i < subscriber->_shared_msgs.size(); ++i)
    {
      subscriber->_shared_msgs[i] =
          std::shared_ptr<Message>((Message*)dds_alloc(sizeof(Message)));
      subscriber->_samples[i] = (void*)subscriber->_shared_msgs[i].get();
    }
    return subscriber;
  }

  ~Subscriber()
  {}

  std::vector<std::shared_ptr<Message>> read()
  {
    std::vector<std::shared_ptr<Message>> msgs;

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
          msgs.push_back(std::shared_ptr<Message>(_shared_msgs[i]));
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

  Subscriber()
  {}
};

} // namespace cyclonedds
} // namespace free_fleet


#endif // SRC__SUBSCRIBER_HPP
