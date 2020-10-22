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

#ifndef INCLUDE__FREE_FLEET__MANAGER_HPP
#define INCLUDE__FREE_FLEET__MANAGER_HPP

#include <memory>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <free_fleet/transport/Middleware.hpp>

namespace free_fleet {

class Manager
{
public:

  using SharedPtr = std::shared_ptr<Manager>;

  ///
  static SharedPtr make(
    const std::string& fleet_name,
    std::shared_ptr<transport::Middleware> middleware,
    std::shared_ptr<rmf_traffic::agv::Graph> graph);

  ///
  void start(uint32_t frequency);

  class Implementation;
private:
  Manager();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MANAGER_HPP
