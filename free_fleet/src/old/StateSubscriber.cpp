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

#include <free_fleet/StateSubscriber.hpp>

#include "messages/FleetMessages.h"
#include "messages/message_utils.hpp"

#include "dds_utils/DDSSubscribeHandler.hpp"

namespace free_fleet {

//==============================================================================

class StateSubscriber::Implementation
{
public:
  using RobotStateSub = dds::DDSSubscribeHandler<FreeFleetData_RobotState, 10>;

  Config _config;
  Participant::SharedPtr _participant;
  RobotStateSub::SharedPtr _robot_state_sub;

  Implementation(
      Config config,
      Participant::SharedPtr participant,
      RobotStateSub::SharedPtr robot_state_sub)
  : _config(std::move(config)),
    _participant(std::move(participant)),
    _robot_state_sub(std::move(robot_state_sub))
  {}

  ~Implementation()
  {}
};

//==============================================================================

void StateSubscriber::Config::print_config() const
{
  printf("STATE SUBSCRIBER CONFIGURATION\n");
  printf("  domain ID: %d\n", domain_id);
  printf("  TOPICS\n");
  printf("    robot state: %s\n", robot_state_topic.c_str());
}

//==============================================================================

StateSubscriber::SharedPtr StateSubscriber::make(
    Config config, Participant::SharedPtr participant)
{
  auto participant_ptr = std::move(participant);
  if (!participant_ptr)
  {
    participant_ptr = Participant::make(config.domain_id);
    if (!participant_ptr)
      return nullptr;
  }

  Implementation::RobotStateSub::SharedPtr robot_state_sub(
      new Implementation::RobotStateSub(
          participant_ptr->id(),
          &FreeFleetData_RobotState_desc,
          config.robot_state_topic));
  
  if (!robot_state_sub->is_ready())
    return nullptr;

  SharedPtr state_subscriber_ptr = SharedPtr(new StateSubscriber);
  state_subscriber_ptr->_pimpl.reset(
      new Implementation(
          std::move(config),
          std::move(participant_ptr),
          std::move(robot_state_sub)));
  return state_subscriber_ptr;
}

//==============================================================================

bool StateSubscriber::read_robot_states(
    std::vector<messages::RobotState>& new_robot_states)
{
  auto robot_states = _pimpl->_robot_state_sub->read();
  if (!robot_states.empty())
  {
    new_robot_states.clear();
    for (size_t i = 0; i < robot_states.size(); ++i)
    {
      messages::RobotState tmp_robot_state;
      convert(*(robot_states[i]), tmp_robot_state);
      new_robot_states.push_back(tmp_robot_state);
    }
    return true;
  }
  return false;
}

//==============================================================================

StateSubscriber::~StateSubscriber()
{}

//==============================================================================

StateSubscriber::StateSubscriber()
{}

//==============================================================================

} // namespace free_fleet
