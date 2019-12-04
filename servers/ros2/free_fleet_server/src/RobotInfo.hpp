/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef FREEFLEETSERVER__SRC__ROBOTINFO_HPP
#define FREEFLEETSERVER__SRC__ROBOTINFO_HPP

#include <rmf_fleet_msgs/msg/robot_state.hpp>

#include "free_fleet/FreeFleet.h"

#include "dds_utils/DDSPublishHandler.hpp"

namespace free_fleet
{

struct RobotInfo
{
  rmf_fleet_msgs::msg::RobotState state_msg;

  using DDSRobotModePub = dds::DDSPublishHandler<FreeFleetData_RobotMode>;
  DDSRobotModePub::SharedPtr dds_mode_request_pub;

  using DDSPathPub = dds::DDSPublishHandler<FreeFleetData_Path>;
  DDSPathPub::SharedPtr dds_path_request_pub;

  using DDSLocationPub = dds::DDSPublishHandler<FreeFleetData_Location>;
  DDSLocationPub::SharedPtr dds_destination_request_pub;
};

} // namespace free_fleet

#endif // FREEFLEETSERVER__SRC__ROBOTINFO_HPP
