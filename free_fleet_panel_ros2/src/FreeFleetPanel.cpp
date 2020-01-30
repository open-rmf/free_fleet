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

#include "FreeFleetPanel.hpp"

#include <rclcpp/node_options.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <QFrame>
#include <QWidget>
#include <QSplitter>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScrollArea>

#include <Eigen/Geometry>

namespace free_fleet
{
namespace viz
{

FreeFleetPanel::FreeFleetPanel(QWidget* parent) :
  rviz_common::Panel(parent)
{
  /// Horizontal layout for path request
  // QHBoxLayout* path_request_layout = new QHBoxLayout;
  // QPushButton* path_selection_button = new QPushButton("Select Path", this);
  // path_display = new QLabel("TODO", this);
  // QPushButton* send_path_request_button = new QPushButton("Send Request", this);
  // path_request_layout->addWidget(path_selection_button);
  // path_request_layout->addWidget(path_display);
  // path_request_layout->addWidget(send_path_request_button);

  /// Vertical layout for sending all types of requests to robots
  // QVBoxLayout* request_layout = new QVBoxLayout;
  // request_layout->addLayout(robot_name_layout);
  // request_layout->addLayout(mode_request_layout);
  // request_layout->addLayout(destination_request_layout);
  // request_layout->addLayout(path_request_layout);

  /// Layouts' creation
  create_fleet_name_group();
  create_robot_name_group();
  create_request_group();

  /// FOR TESTING ONLY, horizontal layout for testing button
  QPushButton* test_push_button = new QPushButton("Publish Marker Array", this);
  test_push_button->setSizePolicy(
      QSizePolicy::Expanding, QSizePolicy::Expanding);

  /// Putting all the other layouts into an overall vertical layout
  QVBoxLayout* vertical_layout = new QVBoxLayout;
  vertical_layout->addWidget(fleet_name_group_box);
  vertical_layout->addWidget(robot_name_group_box);
  vertical_layout->addWidget(request_group_box);
  vertical_layout->addWidget(test_push_button);
  setLayout(vertical_layout);

  connect(
      test_push_button, &QPushButton::clicked, this, 
      &FreeFleetPanel::publish_marker_array);

  // ---------------------------------------------------------------------------
  /// Handle all the ROS related stuff and start spinning
  ros_node = std::make_shared<rclcpp::Node>("free_fleet_panel_ros2_node");

  fleet_state_sub = ros_node->create_subscription<FleetState>(
      "fleet_states", rclcpp::SystemDefaultsQoS(), 
      [&](FleetState::UniquePtr msg)
      {
        fleet_state_cb_fn(std::move(msg));
      });

  marker_array_pub = ros_node->create_publisher<MarkerArray>(
      "marker_array", rclcpp::SystemDefaultsQoS());

  ros_thread = 
      std::thread(&FreeFleetPanel::ros_thread_fn, this);
}

FreeFleetPanel::~FreeFleetPanel()
{
  ros_thread.join();
}

void FreeFleetPanel::ros_thread_fn()
{
  rclcpp::spin(ros_node);
}

void FreeFleetPanel::refresh_fleet_name()
{
  WriteLock fleet_name_lock(fleet_name_mutex);
  QString new_fleet_name = fleet_name_editor->text();

  if (fleet_name.compare(new_fleet_name) != 0)
  {
    fleet_name = new_fleet_name;
    WriteLock robot_states_lock(robot_states_mutex);
    robot_states.clear();

    RCLCPP_INFO(
        ros_node->get_logger(), 
        "fleet_name changed to %s", fleet_name.toStdString().c_str());
  }
}

void FreeFleetPanel::create_fleet_name_group()
{
  fleet_name_group_box = new QGroupBox(tr("Fleet"));
  QGridLayout* layout = new QGridLayout(this);

  fleet_name_editor = new QLineEdit(this);

  QPushButton* refresh_fleet_name_button = 
      new QPushButton(tr("Refresh"), this);

  // number_of_robots_display = new QLabel("0", this);
  test_number = 0;
  number_of_robots_display = 
      new QLabel(std::to_string(test_number).c_str(), this);

  layout->addWidget(new QLabel(tr("Name:"), this), 0, 0, 1, 1);
  layout->addWidget(fleet_name_editor, 0, 1, 1, 4);
  layout->addWidget(refresh_fleet_name_button, 0, 5, 1, 1);
  layout->addWidget(new QLabel(tr("Number of robots:"), this), 1, 0, 1, 3);
  layout->addWidget(number_of_robots_display, 1, 3, 1, 3);

  fleet_name_group_box->setLayout(layout);

  connect(refresh_fleet_name_button, &QPushButton::clicked, this, 
      &FreeFleetPanel::refresh_fleet_name);
}

void FreeFleetPanel::create_robot_name_group()
{
  default_robot_name_selection = tr("Select Robot");
  QStringList robot_name_list = {default_robot_name_selection};

  robot_name_combo = new QComboBox(this);
  robot_name_combo->addItems(robot_name_list);

  QPushButton* move_to_button = new QPushButton(tr("Move To"), this);
  QPushButton* follow_button = new QPushButton(tr("Follow"), this);

  QGridLayout* layout = new QGridLayout(this);
  layout->addWidget(new QLabel(tr("Name:"), this), 0, 0, 1, 1);
  layout->addWidget(robot_name_combo, 0, 1, 1, 3);
  layout->addWidget(move_to_button, 0, 4, 1, 1);
  layout->addWidget(follow_button, 0, 5, 1, 1);
  
  robot_name_group_box = new QGroupBox(tr("Robot"), this);
  robot_name_group_box->setLayout(layout);

  // TODO: button connections
}

void FreeFleetPanel::create_request_group()
{
  request_group_layout = new QVBoxLayout(this);

  create_mode_request_subgroup();
  create_destination_request_subgroup();
  create_path_request_subgroup();

  request_group_box = new QGroupBox(tr("Robot Requests"), this);
  request_group_box->setLayout(request_group_layout);
}

void FreeFleetPanel::create_mode_request_subgroup()
{
  QLabel* mode_subgroup_label = new QLabel(tr("Mode:"), this);

  QStringList mode_list = {"Select Mode", "Pause", "Resume", "Emergency"};
  mode_selection_combo = new QComboBox(this);
  mode_selection_combo->addItems(mode_list);

  QPushButton* send_request_button = new QPushButton(tr("Send"), this);

  QGridLayout* layout = new QGridLayout(this);
  layout->addWidget(mode_subgroup_label, 0, 0, 1, 1);
  layout->addWidget(mode_selection_combo, 1, 0, 1, 3);
  layout->addWidget(new QWidget(this), 1, 3, 1, 2);
  layout->addWidget(send_request_button, 1, 5, 1, 1);

  QWidget* subgroup_widget = new QWidget(this);
  subgroup_widget->setLayout(layout);
  request_group_layout->addWidget(subgroup_widget);

  // TODO: button connections
}

void FreeFleetPanel::create_destination_request_subgroup()
{
  QLabel* destination_subgroup_label = new QLabel(tr("Destination:"), this);

  QPushButton* destination_selection_button = new QPushButton("Select", this);
  QPushButton* send_request_button = new QPushButton(tr("Send"), this);

  destination_x_display = new QLabel("", this);
  destination_y_display = new QLabel("", this);
  destination_yaw_display = new QLabel("", this);

  QGridLayout* layout = new QGridLayout(this);
  layout->addWidget(destination_subgroup_label, 0, 0, 1, 1);

  layout->addWidget(destination_selection_button, 1, 0, 1, 1);
  layout->addWidget(new QWidget(this), 1, 1, 1, 4);
  layout->addWidget(send_request_button, 1, 5, 1, 1);

  layout->addWidget(new QLabel("x:", this), 2, 0, 1, 1);
  layout->addWidget(destination_x_display, 2, 1, 1, 1);
  layout->addWidget(new QLabel("y:", this), 2, 2, 1, 1);
  layout->addWidget(destination_x_display, 2, 3, 1, 1);
  layout->addWidget(new QLabel("yaw:", this), 2, 4, 1, 1);
  layout->addWidget(destination_x_display, 2, 5, 1, 1);

  QWidget* subgroup_widget = new QWidget(this);
  subgroup_widget->setLayout(layout);
  request_group_layout->addWidget(subgroup_widget);

  // TODO: button connections
}

void FreeFleetPanel::create_path_request_subgroup()
{
  QLabel* path_subgroup_label = new QLabel(tr("Path:"), this);

  QPushButton* path_selection_button = new QPushButton("Select", this);
  QPushButton* send_request_button = new QPushButton(tr("Send"), this);

  // setup the scroll area
  path_display = new QLabel("", this);
  QScrollArea* path_display_scroll_area = new QScrollArea(this);
  path_display_scroll_area->setWidget(path_display);

  QGridLayout* layout = new QGridLayout(this);
  layout->addWidget(path_subgroup_label, 0, 0, 1, 1);

  layout->addWidget(path_selection_button, 1, 0, 1, 1);
  layout->addWidget(new QWidget(this), 1, 1, 1, 4);
  layout->addWidget(send_request_button, 1, 5, 1, 1);

  layout->addWidget(path_display_scroll_area, 2, 0, 1, 6);

  // add widget for scroll area
  
  QWidget* subgroup_widget = new QWidget(this);
  subgroup_widget->setLayout(layout);
  request_group_layout->addWidget(subgroup_widget);

  // TODO: button connections
}

void FreeFleetPanel::fleet_state_cb_fn(FleetState::UniquePtr _msg)
{
  QString incoming_fleet_name(std::string(_msg->name).c_str());

  {
    ReadLock fleet_name_lock(fleet_name_mutex);
    if (incoming_fleet_name != fleet_name)
      return;
  }

  MarkerArray array;
  array.markers.clear();
  
  WriteLock robot_states_lock(robot_states_mutex);
  for (const auto& rs : _msg->robots)
  {
    robot_states[rs.name] = rs;

    Marker marker = get_robot_marker_with_shape();
    marker.header.frame_id = "map";
    marker.header.stamp.sec = rs.location.t.sec;
    marker.header.stamp.nanosec = rs.location.t.nanosec;
    marker.ns = rs.name;
    marker.id = 0;
    marker.action = Marker::MODIFY;
    marker.pose.position.x = rs.location.x;
    marker.pose.position.y = rs.location.y;
    marker.pose.position.z = 0.0;

    // Eigen::Quaterniond q;
    // q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
    //     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(rs.location.yaw, Eigen::Vector3d::UnitZ());
    // marker.pose.orientation.x = q.x();
    // marker.pose.orientation.y = q.y();
    // marker.pose.orientation.z = q.z();
    // marker.pose.orientation.w = q.w();
    
    const double yaw = rs.location.yaw;
    const double pitch = 0.0;
    const double roll = 0.0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    marker.pose.orientation.x = cy * cp * sr - sy * sp * cr;
    marker.pose.orientation.y = sy * cp * sr + cy * sp * cr;
    marker.pose.orientation.z = sy * cp * cr - cy * sp * sr;
    marker.pose.orientation.w = cy * cp * cr + sy * sp * sr;

    array.markers.push_back(marker);
  }
  marker_array_pub->publish(array);

  // Update the number of robots registered
  number_of_robots_display->setText(
      QString(std::to_string(robot_states.size()).c_str()));
}

void FreeFleetPanel::publish_marker_array()
{
  MarkerArray msg;

  Marker marker = get_robot_marker_with_shape();
  marker.header.frame_id = "map";
  marker.header.stamp = ros_node->get_clock()->now();
  marker.ns = "free_fleet";
  marker.id = 0;
  marker.action = Marker::MODIFY;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;

  msg.markers.clear();
  msg.markers.push_back(marker);

  marker_array_pub->publish(msg);
}

FreeFleetPanel::Marker FreeFleetPanel::get_robot_marker_with_shape()
{
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = 1.0;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 1.0;
  marker.color.a = 0.75;
  return marker;
}

} // namespace viz
} // namespace free_fleet

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(free_fleet::viz::FreeFleetPanel, rviz_common::Panel)
