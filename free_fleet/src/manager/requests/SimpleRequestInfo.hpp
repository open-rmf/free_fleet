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

#ifndef SRC__MANAGER__REQUESTS__SIMPLEREQUESTINFO_HPP
#define SRC__MANAGER__REQUESTS__SIMPLEREQUESTINFO_HPP

#include <rmf_traffic/Time.hpp>

#include "RequestInfo.hpp"

namespace free_fleet {
namespace manager {

//==============================================================================
template <typename T>
class SimpleRequestInfo : public RequestInfo
{
public:

  using SendRequest = std::function<void(const T&)>;

  using TimeNow = std::function<rmf_traffic::Time()>;

  /// Constructor
  SimpleRequestInfo(
    const T& request,
    SendRequest send_request_fn,
    TimeNow time_now_fn)
  : _request(request),
    _send_request_fn(std::move(send_request_fn)),
    _time_now_fn(std::move(time_now_fn)),
    _init_time(_time_now_fn()),
    _acknowledged_time(std::nullopt)
  {}

  rmf_traffic::Time init_time() const override
  {
    return _init_time;
  }

  std::optional<rmf_traffic::Time> acknowledged() const override
  {
    return _acknowledged_time;
  }

  uint32_t id() const override
  {
    return _request.task_id;
  }

  void send_request() const override
  {
    if (_send_request_fn)
      _send_request_fn(_request);
  }

  void acknowledge_request() override
  {
    if (!_acknowledged_time.has_value())
      _acknowledged_time = _time_now_fn();
  }

  std::pair<RobotInfo::TrackingState, std::size_t> track_robot(
    const RobotInfo& robot_info,
    const messages::RobotState& new_state) const override;

private:
  T _request;
  SendRequest _send_request_fn;
  TimeNow _time_now_fn;
  rmf_traffic::Time _init_time;
  std::optional<rmf_traffic::Time> _acknowledged_time;
};

//==============================================================================
} // namespace manager
} // namespace free_fleet

#endif // SRC__MANAGER__REQUESTS__REQUESTINFO_HPP
