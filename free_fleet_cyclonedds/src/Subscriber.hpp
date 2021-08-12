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

#include <any>
#include <mutex>
#include <memory>
#include <vector>
#include <functional>

#include <dds/dds.h>

namespace free_fleet {
namespace cyclonedds {

//==============================================================================
template <typename Message, size_t MaxSamplesNum = 1>
class Subscriber
: public std::enable_shared_from_this<Subscriber<Message, MaxSamplesNum>>
{
public:

  using SharedPtr = std::shared_ptr<Subscriber<Message, MaxSamplesNum>>;

  static SharedPtr make(
    const dds_entity_t& participant, 
    const dds_topic_descriptor_t* topic_desc, 
    const std::string& topic_name,
    std::optional<std::function<void(const Message& message)>>
      callback = std::nullopt)
  {
    SharedPtr subscriber(new Subscriber<Message, MaxSamplesNum>());

    dds_entity_t topic = dds_create_topic(
      participant, topic_desc, topic_name.c_str(), NULL, NULL);
    if (topic < 0)
    {
      DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));
      return nullptr;
    }
    
    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);

    dds_entity_t reader =
      dds_create_reader(participant, topic, qos, subscriber->_listener);
    if (reader < 0)
    {
      DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
      return nullptr;
    }

    for (std::size_t i = 0; i < subscriber->_shared_msgs.size(); ++i)
    {
      subscriber->_shared_msgs[i] =
          std::shared_ptr<Message>((Message*)dds_alloc(sizeof(Message)));
      subscriber->_raw_msg_samples[i] =
        (void*)subscriber->_shared_msgs[i].get();
    }

    {
      std::lock_guard<std::mutex> s_lock(_s_mutex);
      _s_subscriber_map.insert(std::make_pair(reader, subscriber));
    }

    subscriber->_topic = std::move(topic);
    subscriber->_reader = std::move(reader); 
    std::lock_guard<std::mutex> callback_lock(subscriber->_callback_mutex);
    subscriber->_callback = callback;

    dds_delete_qos(qos);
    return subscriber;
  }

  void set_callback(std::function<void(const Message&)> new_callback)
  {
    std::lock_guard<std::mutex> callback_lock(_callback_mutex);
    _callback = std::move(new_callback);
  }

  ~Subscriber()
  {}

private:

  inline static std::mutex _s_mutex = std::mutex();

  inline static std::unordered_map
    <dds_entity_t, std::shared_ptr<Subscriber<Message, MaxSamplesNum>>>
      _s_subscriber_map = {};

  static void _s_data_available(dds_entity_t reader, void* arg)
  {
    (void)arg;
    std::lock_guard<std::mutex> s_lock(_s_mutex);
    auto it = _s_subscriber_map.find(reader);
    
    if (it == _s_subscriber_map.end())
    {
      DDS_FATAL("_s_data_available: Reader not found or has already expired\n");
      return;
    }
    
    dds_return_t return_code =
      dds_take(
        reader,
        it->second->_raw_msg_samples,
        it->second->_msg_infos,
        MaxSamplesNum,
        MaxSamplesNum);
    if (return_code < 0)
    {
      DDS_FATAL("dds_take: %s\n", dds_strretcode(-return_code));
      return;
    }

    if (return_code > 0)
    {
      for (std::size_t i = 0; i < MaxSamplesNum; ++i)
      {
        if (it->second->_msg_infos[i].valid_data &&
          it->second->_callback.has_value())
        {
          (*it->second->_callback)(*it->second->_shared_msgs[i]);
        }
        else
        {
          return;
        }
      }
    }
  }

  dds_listener_t* _listener;
  dds_entity_t _topic;
  dds_entity_t _reader;

  std::array<std::shared_ptr<Message>, MaxSamplesNum> _shared_msgs;
  void* _raw_msg_samples[MaxSamplesNum];
  dds_sample_info_t _msg_infos[MaxSamplesNum];

  std::mutex _callback_mutex;
  std::optional<std::function<void(const Message&)>> _callback;

  Subscriber()
  {
    _listener = dds_create_listener(NULL);
    dds_lset_data_available(
      _listener, (dds_on_data_available_fn)_s_data_available);
  }
};

//==============================================================================
} // namespace cyclonedds
} // namespace free_fleet

#endif // SRC__SUBSCRIBER_HPP
