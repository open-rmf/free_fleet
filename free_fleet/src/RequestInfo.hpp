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

#ifndef SRC__REQUESTINFO_HPP
#define SRC__REQUESTINFO_HPP

#include <memory>

#include <free_fleet/Manager.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {

template<class RequestType>
class RequestInfo
{
public:
  
  using SharedPtr = std::shared_ptr<RequestInfo>;

  using SendRequest = std::function<void()>;

  /// Gets the request message.
  const RequestType& request()
  {
    return _request;
  }

  /// Calls the provided send request function.
  void send_request()
  {
    _send_request_fn();
  }

  /// The time stamp of when the request was initiated.
  rmf_traffic::Time request_start()
  {
    return _request_started;
  }

private:
  friend class free_fleet::Manager;

  RequestInfo(
    const RequestType& request,
    SendRequest send_request_fn,
    rmf_traffic::Time time_now)
  : _request(request),
    _send_request_fn(std::move(send_request_fn)),
    _request_started(time_now)
  {}

  Request

  RequestType _request;
  SendRequest _send_request_fn;
  rmf_traffic::Time _request_started;
};

} // namespace free_fleet

#endif // SRC__REQUESTINFO_HPP
