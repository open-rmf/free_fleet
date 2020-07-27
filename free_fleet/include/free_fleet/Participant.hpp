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

#ifndef FREE_FLEET__INCLUDE__FREE_FLEET__PARTICIPANT_HPP
#define FREE_FLEET__INCLUDE__FREE_FLEET__PARTICIPANT_HPP

#include <memory>

namespace free_fleet {

class Participant
{

public:

  using SharedPtr = std::shared_ptr<Participant>;

  /// Factory function that creates a free fleet participant on the provided 
  /// domain ID.
  ///
  /// \param[in] domain_id
  ///   Domain ID that the participant will be registered on.
  /// \return
  ///   Shared pointer to a free fleet participant.
  static SharedPtr make(int domain_id);

  /// Returns the participant ID to be used for publishing or subscribing.
  int id() const;

  /// Returns the domain ID that the participant is registered on.
  int domain_id() const;

  /// Destructor
  ~Participant();

  class Implementation;
private:
  Participant();
  std::unique_ptr<Implementation> _pimpl;
};

} // namespace free_fleet

#endif // FREE_FLEET__INCLUDE__FREE_FLEET__PARTICIPANT_HPP
