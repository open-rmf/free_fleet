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

#ifndef SRC__REQUESTS__NAVIGATIONREQUESTINFO_HPP
#define SRC__REQUESTS__NAVIGATIONREQUESTINFO_HPP

#include <functional>

#include <free_fleet/messages/NavigationRequest.hpp>

#include "RequestInfo.hpp"

namespace free_fleet {
namespace requests {

class NavigationRequestInfo : public RequestInfo
{
public:

  using SendRequest = std::function<void (const messages::NavigationRequest&)>;

  /// Constructor
  NavigationRequestInfo(
    const messages::NavigationRequest& request,
    SendRequest send_request_fn,
    rmf_traffic::Time time_now)
  : RequestInfo(time_now, RequestType::NavigationRequest),
    _request(request),
    _send_request_fn(std::move(send_request_fn))
  {}

  /// Gets the request message.
  const messages::NavigationRequest& request() const
  {
    return _request;
  }

  /// Gets the task ID.
  uint32_t id() const final
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
  messages::NavigationRequest _request;
  SendRequest _send_request_fn;
};

} // namespace requests
} // namespace free_fleet

#endif // SRC__REQUESTS__NAVIGATIONREQUESTINFO_HPP
