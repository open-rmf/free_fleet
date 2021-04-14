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

#ifndef SRC__REQUESTS__REQUESTINFO_HPP
#define SRC__REQUESTS__REQUESTINFO_HPP

#include <memory>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/optional.hpp>

namespace free_fleet {
namespace requests {

//==============================================================================
class BaseRequestInfo
{
public:

  using SharedPtr = std::shared_ptr<BaseRequestInfo>;

  /// Enum class to figure out what Request type this info is handling.
  enum class RequestType : uint8_t
  {
    PauseRequest,
    ResumeRequest,
    DockRequest,
    RelocalizationRequest,
    NavigationRequest
  };

  /// Base constructor
  BaseRequestInfo(
    rmf_traffic::Time time_now,
    RequestType request_type)
  : _init_time(time_now),
    _acknowledged_time(rmf_utils::nullopt),
    _request_type(request_type)
  {}

  /// The time stamp of when the request was initiated.
  rmf_traffic::Time init_time() const
  {
    return _init_time;
  }

  /// Whether the request has been acknowledged.
  bool acknowledged() const
  {
    return _acknowledged_time.has_value();
  }

  /// Time stamp of when the request was acknowledged.
  ///
  /// \return
  ///   Returns a nullopt if the request has not been acknowledged.
  rmf_utils::optional<rmf_traffic::Time> acknowledged_time() const
  {
    return _acknowledged_time;
  }

  /// Sets the time that this request was acknowledged.
  void acknowledged_time(rmf_traffic::Time time)
  {
    _acknowledged_time = time;
  }

  /// Gets the type of request
  RequestType request_type() const
  {
    return _request_type;
  }

  /// Gets the task ID of this request.
  virtual uint32_t id() const = 0;

  /// Sends out request.
  virtual void send_request() const = 0;

private:
  rmf_traffic::Time _init_time;
  rmf_utils::optional<rmf_traffic::Time> _acknowledged_time;
  RequestType _request_type;
};

//==============================================================================
template <class T>
class RequestInfo : public BaseRequestInfo
{
public:

  using SendRequest = std::function<void(const T&)>;

  /// Constructor
  RequestInfo(
    RequestType request_type,
    const T& request,
    SendRequest send_request_fn,
    rmf_traffic::Time time_now)
  : BaseRequestInfo(time_now, request_type),
    _request(request),
    _send_request_fn(std::move(send_request_fn))
  {}

  /// Gets the request message.
  const T& request() const
  {
    return _request;
  }

  /// Gets the task ID
  uint32_t id() const override
  {
    return _request.task_id;
  }

  /// Calls the send request function using the request message.
  void send_request() const final
  {
    if (_send_request_fn)
      _send_request_fn(_request);
  }

private:
  T _request;
  SendRequest _send_request_fn;
};

//==============================================================================
} // namespace requests
} // namespace free_fleet

#endif // SRC__REQUESTS__REQUESTINFO_HPP
