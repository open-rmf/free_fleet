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

#include "GraphTracker.hpp"

namespace free_fleet {
namespace agv {

//==============================================================================
GraphTracker::GraphTracker(
  const std::shared_ptr<rmf_traffic::agv::Graph>& graph)
: _graph(graph)
{}

//==============================================================================
void update_estimates(const RobotInfo::SharedPtr& robot_info)
{
  if (robot_info->_state.path.empty())
  {
    robot_info->_lane_occupied = rmf_utils::nullopt;
    
    // if we had prior information
    if (robot_info->_last_known_wp)
    //
  }
  else
  {

  }
}

//==============================================================================
} // namespace agv
} // namespace free_fleet
