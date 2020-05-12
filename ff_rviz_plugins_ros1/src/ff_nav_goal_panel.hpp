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

#ifndef FREE_FLEET_EXAMPLES_ROS1__SRC__FF_NAV_GOAL_PANEL__HPP
#define FREE_FLEET_EXAMPLES_ROS1__SRC__FF_NAV_GOAL_PANEL__HPP

#include <rviz/panel.h>
#include <rviz/display_context.h>
#include <rviz/default_plugin/tools/pose_tool.h>

namespace free_fleet {

class FFNavToolPanel : public rviz::Panel
{

public:

  FFNavToolPanel(QWidget* parent = 0);

  void onInitialize() final;

private Q_SLOTS:

private:

  rviz::DisplayContext* context_;

};

} // namespace free_fleet

#endif // FREE_FLEET_EXAMPLES_ROS1__SRC__FF_NAV_GOAL_PANEL__HPP
