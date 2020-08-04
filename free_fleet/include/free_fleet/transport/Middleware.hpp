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

#ifndef INCLUDE__FREE_FLEET__TRANSPORT__MIDDLEWARE_HPP
#define INCLUDE__FREE_FLEET__TRANSPORT__MIDDLEWARE_HPP

#include <rmf_utils/optional.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

namespace free_fleet {
namespace transport {

class Middleware
{

  using Duration = rmf_traffic::Duration;

  /// Blocking function that requests for a navigation graph over the
  /// middleware.
  ///
  /// \param[in] timeout
  ///   Duration before this request fails and times out, defaults to 10 seconds
  ///
  /// \return
  ///   Received navigation graph with all coordinates in the frame of the robot
  virtual rmf_utils::optional<rmf_traffic::agv::Graph> request_graph(
      Duration timeout = Duration(std::chrono::seconds(10)));

  template <typename Response>
  virtual void start_service(
      std::string service_name, 
      const Response& service_response) = 0;

  virtual bool check_service(
      std::string service_name,
      Duration timeout = Duration(std::chrono::seconds(10))) = 0;

  template <typename Response>
  virtual bool call_service(
      std::string service_name, 
      Response& service_response, 
      Duration timeout = Duration(std::chrono::seconds(10))) = 0;

  template <typename 

  /// Virtual destructor
  ~Middleware() = default;
};

} // namespace transport
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__TRANSPORT__MIDDLEWARE_HPP
