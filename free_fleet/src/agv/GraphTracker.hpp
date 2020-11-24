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

#ifndef SRC__AGV__GRAPHTRACKER_HPP
#define SRC__AGV__GRAPHTRACKER_HPP

#include <memory>

#include <rmf_traffic/agv/Graph.hpp>

#include "RobotInfo.hpp"

namespace free_fleet {
namespace agv {

class GraphTracker
{
public:

  GraphTracker(const std::shared_ptr<rmf_traffic::agv::Graph>& graph);

  /// Using the past information and newest state in RobotInfo, provide
  void update_estimates(const RobotInfo::SharedPtr& robot_info);

private:

  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
};

} // namespace agv
} // namespace free_fleet

#endif // SRC__AGV__GRAPHTRACKER_HPP
