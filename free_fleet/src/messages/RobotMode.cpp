/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <free_fleet/messages/RobotMode.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class RobotMode::Implementation
{
public:

  Mode mode;

  std::string info;
};

//==============================================================================
RobotMode::RobotMode(Mode mode)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{mode, ""}))
{}

//==============================================================================
RobotMode::RobotMode(Mode mode, const std::string& info)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{mode, info}))
{}

//==============================================================================
RobotMode::Mode RobotMode::mode() const
{
  return _pimpl->mode;
}

//==============================================================================
const std::string& RobotMode::info() const
{
  return _pimpl->info;
}

//==============================================================================
bool operator==(const RobotMode& lhs, const RobotMode& rhs)
{
  if (lhs.mode() == rhs.mode() &&
    lhs.info() == rhs.info())
    return true;
  return false;
}

//==============================================================================
bool operator!=(const RobotMode& lhs, const RobotMode& rhs)
{
  return !(lhs == rhs);
}

//==============================================================================
} // namespace messages
} // namespace free_fleet
