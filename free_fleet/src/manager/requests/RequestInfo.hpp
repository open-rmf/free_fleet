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

#ifndef SRC__MANAGER__REQUESTS__REQUESTINFO_HPP
#define SRC__MANAGER__REQUESTS__REQUESTINFO_HPP

#include <utility>
#include <rmf_traffic/Time.hpp>
#include <rmf_utils/optional.hpp>
#include <free_fleet/Types.hpp>
#include <free_fleet/manager/RobotInfo.hpp>

namespace free_fleet {
namespace manager {

//==============================================================================
class RequestInfo
{
public:

  /// Time stamp of when the request was initialized.
  virtual rmf_traffic::Time init_time() const = 0;

  /// Time stamp of when the request was acknowledged.
  ///
  /// \return
  ///   Returns the time stamp of the acknowledgement, if it has not yet been
  ///   acknowledged, a nullopt is returned.
  virtual std::optional<rmf_traffic::Time> acknowledged() const = 0;

  /// Gets the task ID of this request.
  virtual TaskId id() const = 0;

  /// Sends out this request.
  virtual void send_request() const = 0;
 
  /// Acknowledge that this request has been received.
  virtual void acknowledge_request() = 0;

  /// Tracks the robot throughout its navigation graph.
  ///
  /// \return
  ///   Pair consists of the tracking state, as well as the index of the
  ///   component that it is tracked to. If the tracking state is Lost, the
  ///   second value is meaningless.
  virtual std::pair<RobotInfo::TrackingState, std::size_t> track_robot(
    const RobotInfo& robot_info,
    const messages::RobotState& new_state) const = 0;
};

//==============================================================================
} // namespace manager
} // namespace free_fleet

#endif // SRC__MANAGER__REQUESTS__REQUESTINFO_HPP
