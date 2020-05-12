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

#include <iostream>

#include <QLabel>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>

#include <rviz/load_resource.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>

#include "ff_nav_goal_panel.hpp"

namespace free_fleet {

FFNavToolPanel::FFNavToolPanel(QWidget* parent)
: rviz::Panel(parent)
{
  QGridLayout* nav_tool_layout = new QGridLayout;
  nav_tool_layout->addWidget(new QLabel("Select Robot:"), 0, 0, 1, 1);
  nav_tool_layout->addWidget(new QLabel("selection_placeholder"), 0, 1, 1, 3);
  // add box to select available robots

  QIcon destination_icon =
      rviz::loadPixmap("package://ff_rviz_plugins_ros1/icons/destination.svg");
  QPushButton* destination_nav_goal_button =
      new QPushButton(destination_icon, "", this);
  nav_tool_layout->addWidget(destination_nav_goal_button, 0, 4, 1, 1);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(nav_tool_layout);
  setLayout(layout);

  // connect(destination_nav_goal_button, &QPushButton::clicked, this,
  //     &FFNavToolPanel::test_display_context);
}

void FFNavToolPanel::onInitialize()
{
  // if (!vis_manager_)
  //   std::cout << "vis_manager_ is null" << std::endl;
  
  // if (!vis_manager_->getRenderPanel())
  //   std::cout << "render_panel is null" << std::endl;

  // if (!vis_manager_->getRenderPanel()->getManager())
  //   std::cout << "display_context is null" << std::endl;

  context_ = vis_manager_->getRenderPanel()->getManager();
}


} // namespace free_fleet

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(free_fleet::FFNavToolPanel, rviz::Panel)
