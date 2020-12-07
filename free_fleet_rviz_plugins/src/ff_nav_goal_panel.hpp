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

#ifndef FF_RVIZ_PLUGINS_ROS1__SRC__FF_NAV_GOAL_PANEL__HPP
#define FF_RVIZ_PLUGINS_ROS1__SRC__FF_NAV_GOAL_PANEL__HPP

#include <mutex>

#include <QString>
#include <QLineEdit>
#include <QTextEdit>
#include <QGroupBox>

#include <rviz/panel.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <free_fleet/Server.hpp>

namespace free_fleet {

class FFNavToolPanel : public rviz::Panel
{

public:

  FFNavToolPanel(QWidget* parent = 0);

private Q_SLOTS:

  void send_nav_goal();

protected:

private:

  void create_robot_group_box();
  void create_nav_group_box();
  void create_debug_group_box();

  QGroupBox* _robot_group_box;
  QGroupBox* _nav_group_box;
  QGroupBox* _debug_group_box;

  QLineEdit* _fleet_name_edit;
  QLineEdit* _robot_name_edit;
  QTextEdit* _nav_goal_edit;
  QLabel* _debug_label;

  QPushButton* _send_nav_goal_button;

  Server::SharedPtr _free_fleet_server;

  ros::NodeHandle _nh;
  ros::Subscriber _nav_goal_sub;

  std::mutex _nav_goal_mutex;
  geometry_msgs::PoseStamped _nav_goal;

  void update_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

  QString nav_goal_to_qstring(const geometry_msgs::PoseStamped& msg) const;
};

} // namespace free_fleet

#endif // FF_RVIZ_PLUGINS_ROS1__SRC__FF_NAV_GOAL_PANEL__HPP
