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

#include <dds/dds.h>

#include <free_fleet/Participant.hpp>

namespace free_fleet {

//==============================================================================

class Participant::Implementation
{
public:
  dds_entity_t _participant;
  dds_domainid_t _domain_id;

  Implementation(dds_entity_t participant, dds_domainid_t domain_id)
  : _participant(std::move(participant)),
    _domain_id(std::move(domain_id))
  {}

  ~Implementation()
  {
    dds_return_t return_code = dds_delete(_participant);
    if (return_code != DDS_RETCODE_OK)
      DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
};

//==============================================================================

Participant::SharedPtr Participant::make(int domain_id)
{
  dds_domainid_t dds_domain_id = static_cast<dds_domainid_t>(domain_id);
  dds_entity_t dds_participant = 
      dds_create_participant(dds_domain_id, NULL, NULL);
  if (dds_participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-dds_participant));
    return nullptr;
  }

  SharedPtr participant_ptr = SharedPtr(new Participant);
  participant_ptr->_pimpl.reset(new Implementation(
    std::move(dds_participant),
    std::move(dds_domain_id)
  ));
  return participant_ptr;
}

//==============================================================================

int Participant::id() const
{
  return static_cast<int>(_pimpl->_participant);
}

//==============================================================================

int Participant::domain_id() const
{
  return static_cast<int>(_pimpl->_domain_id);
}

//==============================================================================

Participant::~Participant()
{}

//==============================================================================

Participant::Participant()
{}

//==============================================================================

} // namespace free_fleet
