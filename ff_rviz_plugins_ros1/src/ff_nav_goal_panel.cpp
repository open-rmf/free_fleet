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

#include <string>

#include <QLabel>
#include <QSizePolicy>
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
  create_robot_group_box();
  create_nav_group_box();
  create_debug_group_box();

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(_robot_group_box, 0, 0, 1, 1);
  layout->addWidget(_nav_group_box, 1, 0, 6, 1);
  layout->addWidget(_debug_group_box, 7, 0, 1, 1);
  setLayout(layout);

  // connect(_nav_goal_button, &QPushButton::clicked, this,
  //     &FFNavToolPanel::debug);
}

void FFNavToolPanel::onInitialize()
{
  _nav_goal_sub = _nh.subscribe(
          "/move_base_simple/goal", 2, &FFNavToolPanel::update_goal, this);
}

void FFNavToolPanel::debug()
{
}

void FFNavToolPanel::create_robot_group_box()
{
  _robot_group_box = new QGroupBox("Robot Selection");
  QHBoxLayout* layout = new QHBoxLayout;

  _robot_name_edit = new QLineEdit;
  _robot_name_edit->setPlaceholderText("Insert robot name here");
  layout->addWidget(_robot_name_edit);

  _robot_group_box->setLayout(layout);
}

void FFNavToolPanel::create_nav_group_box()
{
  _nav_group_box = new QGroupBox("Navigation");
  QGridLayout* layout = new QGridLayout;

  _nav_goal_edit = new QTextEdit;
  _nav_goal_edit->setReadOnly(true);
  _nav_goal_edit->setPlainText(nav_goal_to_qstring(_nav_goal));

  QPushButton* send_nav_goal_button = new QPushButton("Send Nav Goal");
  QPushButton* send_path_goal_button = new QPushButton("Send Path Goal");

  QSizePolicy size_policy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  size_policy.setHorizontalStretch(0);
  size_policy.setVerticalStretch(0);
  size_policy.setHeightForWidth(
      send_nav_goal_button->sizePolicy().hasHeightForWidth());
  send_nav_goal_button->setSizePolicy(size_policy);
  send_path_goal_button->setSizePolicy(size_policy);

  layout->addWidget(_nav_goal_edit, 0, 0, 6, 3);
  layout->addWidget(send_nav_goal_button, 0, 3, 3, 1);
  layout->addWidget(send_path_goal_button, 3, 3, 3, 1);

  _nav_group_box->setLayout(layout);
}

void FFNavToolPanel::create_debug_group_box()
{
  _debug_group_box = new QGroupBox("Debug");
  QHBoxLayout* layout = new QHBoxLayout;

  _debug_label = new QLabel("Debugging started...");
  layout->addWidget(_debug_label);

  _debug_group_box->setLayout(layout);
}

void FFNavToolPanel::update_goal(
    const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::unique_lock<std::mutex> nav_goal_lock(_nav_goal_mutex);
  _nav_goal = *msg;
  _nav_goal_edit->setPlainText(nav_goal_to_qstring(_nav_goal));
}

QString FFNavToolPanel::nav_goal_to_qstring(
    const geometry_msgs::PoseStamped& msg) const
{
  std::ostringstream ss;
  ss <<
      "Position:" <<
      "\n    x: " << std::to_string(msg.pose.position.x) <<
      "\n    y: " << std::to_string(msg.pose.position.y) <<
      "\n    z: " << std::to_string(msg.pose.position.z) <<
      "\nOrientation:" <<
      "\n    x: " << std::to_string(msg.pose.orientation.x) <<
      "\n    y: " << std::to_string(msg.pose.orientation.y) <<
      "\n    z: " << std::to_string(msg.pose.orientation.z) <<
      "\n    w: " << std::to_string(msg.pose.orientation.w) << std::endl;
  return QString::fromStdString(ss.str());
}

} // namespace free_fleet

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(free_fleet::FFNavToolPanel, rviz::Panel)
