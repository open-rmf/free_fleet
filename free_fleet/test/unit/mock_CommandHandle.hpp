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

#ifndef TEST__UNIT__MOCK_COMMANDHANDLE_HPP
#define TEST__UNIT__MOCK_COMMANDHANDLE_HPP

#include <free_fleet/client/CommandHandle.hpp>

namespace free_fleet {

class MockCommandHandle : public client::CommandHandle
{
public:

  MockCommandHandle()
  {}

  void relocalize(
    const free_fleet::messages::Location&,
    RequestCompleted) final
  {}

  void follow_new_path(
      const std::vector<free_fleet::messages::Waypoint>&,
      RequestCompleted) final
  {}

  void stop() final
  {}

  void resume() final
  {}

  void dock(
    const std::string&,
    RequestCompleted) final
  {}

};

} // namespace free_fleet

#endif // TEST__UNIT__MOCK_COMMANDHANDLE_HPP
