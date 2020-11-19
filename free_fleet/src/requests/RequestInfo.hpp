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

#include <free_fleet/Manager.hpp>

namespace free_fleet {

class RequestInfo
{
public:

  using SharedPtr = std::shared_ptr<RequestInfo>;

  /// Base constructor
  RequestInfo(rmf_traffic::Time time_now)
  : _request_start_time(time_now)
  {}

  /// The time stamp of when the request was initiated.
  rmf_traffic::Time request_start_time()
  {
    return _request_start_time;
  }

private:
  rmf_traffic::Time _request_start_time;
}

} // namespace free_fleet

#endif // SRC__REQUESTS__REQUESTINFO_HPP
