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

#ifndef FREE_FLEET__INCLUDE__FREE_FLEET__STATESUBSCRIBER_HPP
#define FREE_FLEET__INCLUDE__FREE_FLEET__STATESUBSCRIBER_HPP

#include <string>
#include <memory>

#include <free_fleet/Participant.hpp>
#include <free_fleet/messages/RobotState.hpp>

namespace free_fleet {

class StateSubscriber
{

public:

  /// Configuration for setting up a StateSubscriber, defines the DDS domain,
  /// and topic names for receiving states.
  struct Config
  {
    int domain_id = 42;
    std::string robot_state_topic = "robot_state";

    /// Prints the values of this StateSubscriber configuration.
    void print_config() const;
  };

  using SharedPtr = std::shared_ptr<StateSubscriber>;
  
  /// Factory function that creates an instance of the Free Fleet Request 
  /// Publisher.
  ///
  /// \param[in] config
  ///   Configuration that sets up the request publisher.
  /// \param[in] participant
  ///   Shared pointer to a participant, that can be used to create the
  ///   subscriber, instead of re-initializing a participant.
  /// \return
  ///   Shared pointer to the request publisher, nullptr if creation failed.
  static SharedPtr make(
      Config config, Participant::SharedPtr participant = nullptr);

  /// Attempts to read new incoming robot states sent by free fleet clients
  /// over DDS.
  ///
  /// \param[out] new_robot_states
  ///   A vector of new incoming robot states sent by clients to update the
  ///   fleet management system.
  /// \return
  ///   True if new robot states were received, false otherwise.
  bool read_robot_states(std::vector<messages::RobotState>& new_robot_states);

  /// Destructor
  ~StateSubscriber();

  class Implementation;
private:
  StateSubscriber();
  std::unique_ptr<Implementation> _pimpl;
};

} // namespace free_fleet

#endif // FREE_FLEET__INCLUDE__FREE_FLEET__STATESUBSCRIBER_HPP
