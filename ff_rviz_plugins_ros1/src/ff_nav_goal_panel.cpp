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

#include <free_fleet/ServerConfig.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>
#include <free_fleet/messages/RobotState.hpp>

#include "ff_nav_goal_panel.hpp"
#include "utilities.hpp"

namespace free_fleet {

//==============================================================================

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

  connect(_send_nav_goal_button, &QPushButton::clicked, this,
      &FFNavToolPanel::send_nav_goal);

  ServerConfig default_server_conf;
  _free_fleet_server = Server::make(default_server_conf);
  if (!_free_fleet_server)
  {
    _debug_label->setText("Free Fleet server unable to start...");
    return;
  }

  _nav_goal_sub = _nh.subscribe(
      "/move_base_simple/goal", 2, &FFNavToolPanel::update_goal, this);
}

//==============================================================================

void FFNavToolPanel::send_nav_goal()
{
  std::string fleet_name = _fleet_name_edit->text().toStdString();
  std::string robot_name = _robot_name_edit->text().toStdString();

  messages::Location nav_goal_location;
  messages::DestinationRequest destination_request;

  {
    std::unique_lock<std::mutex> nav_goal_lock(_nav_goal_mutex);
    nav_goal_location = {
      static_cast<int32_t>(_nav_goal.header.stamp.sec),
      _nav_goal.header.stamp.nsec,
      static_cast<float>(_nav_goal.pose.position.x),
      static_cast<float>(_nav_goal.pose.position.y),
      static_cast<float>(get_yaw_from_quat(_nav_goal.pose.orientation)),
      ""
    };
    destination_request = {
      std::move(fleet_name),
      std::move(robot_name),
      std::move(nav_goal_location),
      generate_random_task_id(20)
    };
  }

  if (!_free_fleet_server->send_destination_request(destination_request))
  {
    std::string debug_str = "Failed to send navigation request...";
    _debug_label->setText(QString::fromStdString(debug_str));
    return;
  }
}

//==============================================================================

void FFNavToolPanel::create_robot_group_box()
{
  _robot_group_box = new QGroupBox("Robot Selection");
  QGridLayout* layout = new QGridLayout;

  _fleet_name_edit = new QLineEdit;
  _fleet_name_edit->setPlaceholderText("enter fleet name here");

  _robot_name_edit = new QLineEdit;
  _robot_name_edit->setPlaceholderText("enter robot name here");

  layout->addWidget(new QLabel("Fleet:"), 0, 0, 1, 1);
  layout->addWidget(_fleet_name_edit, 0, 1, 1, 2);
  layout->addWidget(new QLabel("Robot:"), 1, 0, 1, 1);
  layout->addWidget(_robot_name_edit, 1, 1, 1, 2);
  _robot_group_box->setLayout(layout);
}

//==============================================================================

void FFNavToolPanel::create_nav_group_box()
{
  _nav_group_box = new QGroupBox("Navigation");
  QGridLayout* layout = new QGridLayout;

  _nav_goal_edit = new QTextEdit;
  _nav_goal_edit->setReadOnly(true);
  _nav_goal_edit->setPlainText(nav_goal_to_qstring(_nav_goal));

  _send_nav_goal_button = new QPushButton("Send Nav Goal");

  QSizePolicy size_policy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  size_policy.setHorizontalStretch(0);
  size_policy.setVerticalStretch(0);
  size_policy.setHeightForWidth(
      _send_nav_goal_button->sizePolicy().hasHeightForWidth());
  _send_nav_goal_button->setSizePolicy(size_policy);

  layout->addWidget(_nav_goal_edit, 0, 0, 6, 3);
  layout->addWidget(_send_nav_goal_button, 0, 3, 6, 1);

  _nav_group_box->setLayout(layout);
}

//==============================================================================

void FFNavToolPanel::create_debug_group_box()
{
  _debug_group_box = new QGroupBox("Debug");
  QHBoxLayout* layout = new QHBoxLayout;

  _debug_label = new QLabel("Panel started...");
  layout->addWidget(_debug_label);

  _debug_group_box->setLayout(layout);
}

//==============================================================================

void FFNavToolPanel::update_goal(
    const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::unique_lock<std::mutex> nav_goal_lock(_nav_goal_mutex);
  _nav_goal = *msg;
  _nav_goal_edit->setPlainText(nav_goal_to_qstring(_nav_goal));
}

//==============================================================================

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

//==============================================================================

} // namespace free_fleet

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(free_fleet::FFNavToolPanel, rviz::Panel)
