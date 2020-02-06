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

#ifndef FREE_FLEET_PANEL_ROS2__SRC__ROBOTVIEWCONTROLLER_HPP
#define FREE_FLEET_PANEL_ROS2__SRC__ROBOTVIEWCONTROLLER_HPP

#include <rviz_default_plugins/view_controllers/xy_orbit/xy_orbit_view_controller.hpp>

#include <QWidget>

namespace free_fleet
{
namespace viz
{

class RobotViewController : 
    public rviz_default_plugins::view_controllers::XYOrbitViewController
{

Q_OBJECT

public:

  RobotViewController();

  ~RobotViewController();

private:

};

} // namespace viz
} // namespace free_fleet

#endif // FREE_FLEET_PANEL_ROS2__SRC__ROBOTVIEWCONTROLLER_HPP
