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

#ifndef INCLUDE__FREE_FLEET__MESSAGES__ROBOTSTATE_HPP
#define INCLUDE__FREE_FLEET__MESSAGES__ROBOTSTATE_HPP

#include <string>
#include <vector>
#include <optional>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

#include <free_fleet/Types.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/RobotMode.hpp>

namespace free_fleet {
namespace messages {

//==============================================================================
class RobotState
{
public:

  /// Constructor.
  /// \param[in] time
  ///   The time stamp of this state. 
  ///
  /// \param[in] name
  ///   The name of this robot. A std::invalid_argument will be thrown if this
  ///   is empty.
  ///
  /// \param[in] model
  ///   The model name of this robot.
  ///   
  /// \param[in] task_id
  ///   The taks id of the task that the robot is currently performing. If the
  ///   robot has completed its previous task and is idle, this should be a
  ///   nullopt.
  ///
  /// \param[in] mode
  ///   The current mode of the robot.
  ///
  /// \param[in] battery_percent
  ///   The current battery percentage of the robot. A std::invalid_argument
  ///   will be thrown if this value is not between 0.0 (empty) and 1.0 (full).
  ///
  /// \param[in] location
  ///   The current location of the robot.
  ///
  /// \param[in] target_path_index
  ///   The target path index for the waypoint, which the robot is currently
  ///   navigating towards to.
  RobotState(
    rmf_traffic::Time time,
    const std::string& name,
    const std::string& model,
    std::optional<TaskId> task_id,
    const RobotMode& mode,
    double battery_percent,
    const Location& location,
    std::size_t target_path_index);
  
  /// Gets the current time stamp of this state.
  rmf_traffic::Time time() const;

  /// Gets the robot name.
  const std::string& name() const;

  /// Gets the robot model.
  const std::string& model() const;

  /// Gets the current task id. If the robot has completed its task and is idle,
  /// returns a nullopt.
  std::optional<TaskId> task_id() const;

  /// Gets the robot mode.
  const RobotMode& mode() const;

  /// Gets the robot battery percentage.
  double battery_percent() const;

  /// Gets the robot location.
  const Location& location() const;

  /// Gets the target path index for the waypoint, which the robot is currently
  /// navigating towards to.
  std::size_t target_path_index() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Comparing operators.
bool operator==(const RobotState& lhs, const RobotState& rhs);

bool operator!=(const RobotState& lhs, const RobotState& rhs);

//==============================================================================
} // namespace messages
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MESSAGES__ROBOTSTATE_HPP
