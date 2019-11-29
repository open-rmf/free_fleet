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

#ifndef FREEFLEETCLIENT__SRC__DDSSUBSCRIBEHANDLER_HPP
#define FREEFLEETCLIENT__SRC__DDSSUBSCRIBEHANDLER_HPP

#include <memory>

#include <dds/dds.h>

namespace free_fleet
{
namespace dds
{

template <typename Message>
class DDSSubscribeHandler
{
public:

  using SharedPtr = std::shared_ptr<DDSSubscribeHandler>;
  using MessageSharedPtr = std::shared_ptr<Message>;
  using MessageConstSharedPtr = std::shared_ptr<const Message>;

private:

  dds_return_t return_code;

  const dds_topic_descriptor_t* topic_desc;

  dds_entity_t topic;
  
  dds_entity_t reader;
  
  MessageSharedPtr shared_msg;

  void* samples[1];

  dds_sample_info_t infos[1];

  bool ready;

public:

  /// 
  DDSSubscribeHandler(
      const dds_entity_t& _participant, 
      const dds_topic_descriptor_t* _topic_desc, 
      const std::string& _topic_name) :
    topic_desc(_topic_desc)
  {
    ready = false;

    topic = dds_create_topic(
        _participant, _topic_desc, _topic_name.c_str(), NULL, NULL);
    if (topic < 0)
    {
      DDS_FATAL(
          "dds_create_topic: %s\n", dds_strretcode(-topic));
      return;
    }

    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    reader = dds_create_reader(_participant, topic, qos, NULL);
    if (reader < 0)
    {
      DDS_FATAL(
          "dds_create_reader: %s\n", dds_strretcode(-reader));
      return;
    }
    dds_delete_qos(qos);

    shared_msg = MessageSharedPtr((Message*)dds_alloc(sizeof(Message)));
    samples[0] = (void*)shared_msg.get();

    ready = true;
  }

  /// 
  ~DDSSubscribeHandler()
  {}

  /// Checks if all the DDS items have been set up correctly
  ///
  bool is_ready()
  {
    return ready;
  }

  /// This allows downstream code to edit the contents of the message but not
  /// delete it. This is not the desired behavior but it may be the best we
  /// to work with C libraries.
  ///
  MessageConstSharedPtr read()
  {
    if (!is_ready())
      return nullptr;
    
    return_code = dds_take(reader, samples, infos, 1, 1);
    if (return_code < 0)
    {
      DDS_FATAL("dds_take: %s\n", dds_strretcode(-return_code));
      return nullptr;
    }

    if ((return_code > 0) && (infos[0].valid_data))
    {
      return MessageConstSharedPtr(shared_msg);
    }
    return nullptr;
  }

};

} // namespace dds
} // namespace free_fleet


#endif // FREEFLEETCLIENT__SRC__DDSSUBSCRIBEHANDLER_HPP
