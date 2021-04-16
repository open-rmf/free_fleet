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

#ifndef SRC__REQUESTS__REQUESTINFO_HPP
#define SRC__REQUESTS__REQUESTINFO_HPP

#include <utility>
#include <rmf_traffic/Time.hpp>
#include <rmf_utils/optional.hpp>
#include <free_fleet/agv/RobotInfo.hpp>

namespace free_fleet {
namespace requests {

//==============================================================================
class RequestInfo
{
public:

  /// Time stamp of when the request was initialized.
  virtual rmf_traffic::Time init_time() const = 0;

  /// Whether the request has been acknowledged by the robot.
  virtual bool acknowledged() const = 0;

  /// Time stamp of when the request was acknowledged.
  ///
  /// \return
  ///   Returns a nullopt if the request has not been acknowledged.
  virtual rmf_utils::optional<rmf_traffic::Time> acknowledged_time() const = 0;

  /// Sets the time that this request was acknowledged.
  virtual void acknowledged_time(rmf_traffic::Time time) = 0;
  
  /// Gets the task ID of this request.
  virtual uint32_t id() const = 0;

  /// Sends oout this request.
  virtual void send_request() const = 0;
  
  /// Tracks the robot throughout its navigation graph.
  ///
  /// \return
  ///   Pair consists of the tracking state, as well as the index of the
  ///   component that it is tracked to. If the tracking state is Lost, the
  ///   second value is meaningless.
  virtual std::pair<agv::RobotInfo::TrackingState, std::size_t> track_robot(
    const agv::RobotInfo& robot_info,
    const messages::RobotState& new_state) const = 0;
};

//==============================================================================
} // namespace requests
} // namespace free_fleet

#endif // SRC__REQUESTS__REQUESTINFO_HPP
