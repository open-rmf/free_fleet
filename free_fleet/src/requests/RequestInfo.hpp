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

namespace free_fleet {
namespace requests {

class RequestInfo
{
public:

  using SharedPtr = std::shared_ptr<RequestInfo>;

  /// Enum class to figure out what Request type this info is handling.
  enum class RequestType : uint8_t
  {
    ModeRequest,
    NavigationRequest,
    RelocalizationRequest
  };

  /// Base constructor
  RequestInfo(
    rmf_traffic::Time time_now,
    RequestType request_type)
  : _request_start_time(time_now),
    _request_type(request_type)
  {}

  /// The time stamp of when the request was initiated.
  rmf_traffic::Time request_start_time() const
  {
    return _request_start_time;
  }

  /// Gets the type of request
  RequestType request_type() const
  {
    return _request_type;
  }

  /// 
  virtual void send_request() const = 0;

private:
  rmf_traffic::Time _request_start_time;
  RequestType _request_type;
};

} // namespace requests
} // namespace free_fleet

#endif // SRC__REQUESTS__REQUESTINFO_HPP
