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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__ROBOTMODE_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__ROBOTMODE_HPP

#include <cstdint>

#include <rmf_utils/impl_ptr.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class RobotMode
{
public:

  /// Types of modes of the robot.
  enum class Mode : uint32_t
  {
    Idle,
    Charging,
    Moving,
    Paused,
    Waiting,
    Emergency,
    Docking,
    Error,
    Undefined,
    Custom
  };

  /// Constructor
  ///
  /// \param[in] mode
  ///   The current robot mode.
  RobotMode(Mode mode);

  /// Constructor
  ///
  /// \param[in] mode
  ///   The current robot mode.
  ///
  /// \param[in] info
  ///   Any additional information about this mode that is relevant.
  RobotMode(Mode mode, const std::string& info);

  /// Gets the mode.
  Mode mode() const;

  /// Gets the additional information if any. Returns an empty string if not
  /// set.
  const std::string& info() const;
  
  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Comparing Operators.
bool operator==(const RobotMode& lhs, const RobotMode& rhs);

bool operator!=(const RobotMode& lhs, const RobotMode& rhs);

//==============================================================================
} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__ROBOTMODE_HPP
