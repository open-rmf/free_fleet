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

#include <random>

#include "FreeFleetPanel.hpp"

#include <rclcpp/node_options.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <QtGlobal>

#include <QFrame>
#include <QWidget>
#include <QSplitter>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScrollArea>

#include <Eigen/Geometry>

#include "rviz_common/load_resource.hpp"

namespace free_fleet
{
namespace viz
{

//=============================================================================

FreeFleetPanel::FreeFleetPanel(QWidget* parent) :
  rviz_common::Panel(parent)
{
  create_fleet_name_group();

  create_robot_name_group();

  create_request_group();

  /// Putting all the other layouts into an overall vertical layout
  QVBoxLayout* vertical_layout = new QVBoxLayout;
  vertical_layout->addWidget(fleet_name_group_box);
  vertical_layout->addWidget(robot_name_group_box);
  vertical_layout->addWidget(request_group_box);
  setLayout(vertical_layout);

  initialize_ros();
}

//=============================================================================

FreeFleetPanel::~FreeFleetPanel()
{
  ros_thread.join();
}

//=============================================================================

void FreeFleetPanel::refresh_fleet_name()
{
  WriteLock fleet_name_lock(fleet_name_mutex);
  QString new_fleet_name = fleet_name_editor->text();

  if (fleet_name.compare(new_fleet_name) != 0)
  {
    fleet_name = new_fleet_name;

    clear_robot_states();

    RCLCPP_INFO(
        ros_node->get_logger(), 
        "fleet_name changed to %s", fleet_name.toStdString().c_str());
  }
}

//=============================================================================

void FreeFleetPanel::refresh_robot_name_list()
{
  ReadLock robot_states_lock(robot_states_mutex);
  QStringList current_robot_name_list = {};
  for (auto it = robot_states.begin(); it != robot_states.end(); ++it)
  {
    current_robot_name_list << QString((it->first).c_str());
  }
  current_robot_name_list.sort();

  QStringList full_list_with_placeholder = {robot_name_placeholder};
  full_list_with_placeholder << current_robot_name_list;

  robot_name_combo->clear();
  robot_name_combo->addItems(full_list_with_placeholder);
}

//=============================================================================

void FreeFleetPanel::move_to_robot()
{
  RCLCPP_INFO(
      ros_node->get_logger(), "move_to_robot has yet to be implemented.");
}

//=============================================================================

void FreeFleetPanel::follow_robot()
{
  RCLCPP_INFO(
      ros_node->get_logger(), "follow_robot has yet to be implemented.");
}

//=============================================================================

void FreeFleetPanel::send_robot_mode_request()
{
  ModeRequest msg;

  {
    ReadLock fleet_name_lock(fleet_name_mutex);
    msg.fleet_name = fleet_name.toStdString();
  }

  const QString current_selected_rn = robot_name_combo->currentText();
  msg.robot_name = current_selected_rn.toStdString();

  // QStringList mode_list = {"Select Mode", "Pause", "Resume", "Emergency"};
  switch (mode_selection_combo->currentIndex())
  {
    case 1:
      msg.mode.mode = RobotMode::MODE_PAUSED;
      break;
    case 2:
      msg.mode.mode = RobotMode::MODE_MOVING;
      break;
    case 3:
      msg.mode.mode = RobotMode::MODE_EMERGENCY;
      break;
    default:
      return;
  }

  msg.task_id = generate_new_random_task_id();

  mode_request_pub->publish(msg);
}

//=============================================================================

void FreeFleetPanel::select_robot_destination()
{
  RCLCPP_INFO(
      ros_node->get_logger(), 
      "select_robot_destination has yet to be implemented.");
}

//=============================================================================

void FreeFleetPanel::send_robot_destination_request()
{
  RCLCPP_INFO(
      ros_node->get_logger(), 
      "send_robot_destination_request has yet to be implemented.");
}

//=============================================================================

void FreeFleetPanel::select_robot_path()
{
  RCLCPP_INFO(
      ros_node->get_logger(), 
      "select_robot_path has yet to be implemented.");
}

//=============================================================================

void FreeFleetPanel::send_robot_path_request()
{
  RCLCPP_INFO(
      ros_node->get_logger(), 
      "send_robot_path_request has yet to be implemented.");
}

//=============================================================================

// implementation from 
// https://stackoverflow.com/questions/440133/how-do-i-create-a-random-alpha-numeric-string-in-c
std::string FreeFleetPanel::generate_new_random_task_id(
    unsigned int num_chars) const
{  
  auto randchar = []() -> char
  {
    static std::string character_list = 
        "0123456789"
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    thread_local static std::mt19937 rg{std::random_device{}()};
    thread_local static std::uniform_int_distribution<std::string::size_type> 
        pick(0, sizeof(character_list) - 2);
    return pick(rg);
  };

  std::string str(num_chars, 0);
  std::generate_n(str.begin(), num_chars, randchar);
  return str;
}

//=============================================================================

void FreeFleetPanel::initialize_ros()
{
  setbuf(stdout, NULL);

  ros_node = std::make_shared<rclcpp::Node>("free_fleet_panel_ros2_node");

  fleet_state_sub = ros_node->create_subscription<FleetState>(
      "fleet_states", rclcpp::SystemDefaultsQoS(), 
      [&](FleetState::UniquePtr msg)
      {
        fleet_state_cb_fn(std::move(msg));
      });

  marker_array_pub = ros_node->create_publisher<MarkerArray>(
      "marker_array", rclcpp::SystemDefaultsQoS());

  mode_request_pub = ros_node->create_publisher<ModeRequest>(
      "mode_request", rclcpp::SystemDefaultsQoS());
      
  path_request_pub = ros_node->create_publisher<PathRequest>(
      "path_request", rclcpp::SystemDefaultsQoS());

  destination_request_pub = ros_node->create_publisher<DestinationRequest>(
      "destination_request", rclcpp::SystemDefaultsQoS());

  using namespace std::chrono_literals;

  refresh_values_timer = ros_node->create_wall_timer(
      500ms, std::bind(&FreeFleetPanel::refresh_values, this));

  ros_thread = 
      std::thread(&FreeFleetPanel::ros_thread_fn, this);
}

//=============================================================================

void FreeFleetPanel::ros_thread_fn()
{
  rclcpp::spin(ros_node);
}

//=============================================================================

void FreeFleetPanel::create_fleet_name_group()
{
  fleet_name_group_box = new QGroupBox(tr("Fleet"));
  QGridLayout* layout = new QGridLayout(this);

  fleet_name_editor = new QLineEdit(this);

  QIcon refresh_icon = 
      rviz_common::loadPixmap(
          "package://free_fleet_panel_ros2/media/Refresh-icon.png");
  QPushButton* refresh_fleet_name_button = 
      new QPushButton(refresh_icon, "", this);

  {
    WriteLock robot_states_lock(robot_states_mutex);
    robot_states.clear();
    number_of_robots_display = 
        new QLabel(std::to_string(robot_states.size()).c_str(), this);
  }

  layout->addWidget(new QLabel(tr("Name:"), this), 0, 0, 1, 4);
  layout->addWidget(fleet_name_editor, 0, 4, 1, 12);
  layout->addWidget(new QWidget(this), 0, 16, 1, 1);
  layout->addWidget(refresh_fleet_name_button, 0, 17, 1, 1);

  layout->addWidget(new QLabel(tr("Number of robots:"), this), 1, 0, 1, 9);
  layout->addWidget(number_of_robots_display, 1, 9, 1, 9);

  fleet_name_group_box->setLayout(layout);

  connect(refresh_fleet_name_button, &QPushButton::clicked, this, 
      &FreeFleetPanel::refresh_fleet_name);
}

//=============================================================================

void FreeFleetPanel::create_robot_name_group()
{
  robot_name_placeholder = tr("Select Robot");
  QStringList robot_name_list = {robot_name_placeholder};

  robot_mode_placeholder = tr("None");

  robot_name_combo = new QComboBox(this);
  robot_name_combo->addItems(robot_name_list);

  QIcon refresh_icon = 
      rviz_common::loadPixmap(
          "package://free_fleet_panel_ros2/media/Refresh-icon.png");
  QPushButton* refresh_robot_list_button = 
      new QPushButton(refresh_icon, "", this);

  QPushButton* move_to_button = new QPushButton(tr("Move To"), this);
  QPushButton* follow_button = new QPushButton(tr("Follow"), this);

  current_robot_mode = new QLabel(robot_mode_placeholder, this);

  QGridLayout* layout = new QGridLayout(this);
  layout->addWidget(new QLabel(tr("Name:"), this), 0, 0, 1, 4);
  layout->addWidget(robot_name_combo, 0, 4, 1, 12);
  layout->addWidget(new QWidget(this), 0, 16, 1, 1);
  layout->addWidget(refresh_robot_list_button, 0, 17, 1, 1);

  layout->addWidget(new QLabel(tr("Mode:"), this), 1, 0, 1, 4);
  layout->addWidget(current_robot_mode, 1, 4, 1, 4);
  layout->addWidget(new QWidget(this), 1, 8, 1, 4);
  layout->addWidget(move_to_button, 1, 12, 1, 3);
  layout->addWidget(follow_button, 1, 15, 1, 3);
  
  robot_name_group_box = new QGroupBox(tr("Robot"), this);
  robot_name_group_box->setLayout(layout);

  connect(refresh_robot_list_button, &QPushButton::clicked, this,
      &FreeFleetPanel::refresh_robot_name_list);
  connect(move_to_button, &QPushButton::clicked, this,
      &FreeFleetPanel::move_to_robot);
  connect(follow_button, &QPushButton::clicked, this,
      &FreeFleetPanel::follow_robot);
  connect(
      robot_name_combo,
      QOverload<const QString&>::of(&QComboBox::currentTextChanged),
      [=](const QString& robot_name)
      {
        selected_robot_name(robot_name);
      });
}

//=============================================================================

void FreeFleetPanel::clear_robot_states()
{
  WriteLock robot_states_lock(robot_states_mutex);

  MarkerArray array;
  array.markers.clear();

  for (auto it = robot_states.begin(); it != robot_states.end(); ++it)
  {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros_node->get_clock()->now();
    marker.ns = it->second.name;
    marker.id = 0;
    marker.action = Marker::DELETE;
    array.markers.push_back(marker);
  }
  marker_array_pub->publish(array);
  
  robot_states.clear();
  number_of_robots_display->setText(
      std::to_string(robot_states.size()).c_str());

  QStringList robot_name_list = {robot_name_placeholder};
  robot_name_combo->clear();
  robot_name_combo->addItems(robot_name_list);

  current_robot_mode->setText(robot_mode_placeholder);
}

//=============================================================================

void FreeFleetPanel::selected_robot_name(const QString& _robot_name)
{
  if (_robot_name.compare(robot_name_placeholder) == 0 ||
      _robot_name.compare("") == 0)
    return;

  RCLCPP_INFO(
      ros_node->get_logger(), 
      "robot name selected: %s", 
      (_robot_name.toStdString()).c_str());
}

//=============================================================================

void FreeFleetPanel::display_robot_mode()
{
  const QString current_selected_rn = robot_name_combo->currentText();
  if (current_selected_rn.compare(robot_name_placeholder) == 0 ||
      current_selected_rn.compare("") == 0)
    current_robot_mode->setText(robot_mode_placeholder);
  
  const std::string str_rn = current_selected_rn.toStdString();

  ReadLock robot_states_lock(robot_states_mutex);
  auto it = robot_states.find(str_rn);
  if (it != robot_states.end())
  {
    switch(it->second.mode.mode) {
      case RobotMode::MODE_IDLE:
        current_robot_mode->setText(QString(tr("Idle")));
        break;
      case RobotMode::MODE_CHARGING:
        current_robot_mode->setText(QString(tr("Charging")));
        break;
      case RobotMode::MODE_MOVING:
        current_robot_mode->setText(QString(tr("Moving")));
        break;
      case RobotMode::MODE_PAUSED:
        current_robot_mode->setText(QString(tr("Paused")));
        break;
      case RobotMode::MODE_WAITING:
        current_robot_mode->setText(QString(tr("Waiting")));
        break;
      case RobotMode::MODE_EMERGENCY:
        current_robot_mode->setText(QString(tr("Emergency")));
        break;
      case RobotMode::MODE_GOING_HOME:
        current_robot_mode->setText(QString(tr("Going Home")));
        break;
      case RobotMode::MODE_DOCKING:
        current_robot_mode->setText(QString(tr("Docking")));
        break;
      default:
        current_robot_mode->setText(QString(tr("Undefined")));
    }
  }
}

//=============================================================================

void FreeFleetPanel::create_request_group()
{
  request_group_layout = new QVBoxLayout(this);

  create_mode_request_subgroup();
  create_destination_request_subgroup();
  create_path_request_subgroup();

  request_group_box = new QGroupBox(tr("Robot Requests"), this);
  request_group_box->setLayout(request_group_layout);
}

//=============================================================================

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

  connect(send_request_button, &QPushButton::clicked, this,
      &FreeFleetPanel::send_robot_mode_request);
}

//=============================================================================

void FreeFleetPanel::create_destination_request_subgroup()
{
  QLabel* destination_subgroup_label = new QLabel(tr("Destination:"), this);

  QPushButton* select_destination_button = new QPushButton("Select", this);
  QPushButton* send_request_button = new QPushButton(tr("Send"), this);

  coordinate_placeholder = tr("null");

  destination_x_display = new QLabel(coordinate_placeholder, this);
  destination_y_display = new QLabel(coordinate_placeholder, this);
  destination_yaw_display = new QLabel(coordinate_placeholder, this);

  QGridLayout* layout = new QGridLayout(this);
  layout->addWidget(destination_subgroup_label, 0, 0, 1, 2);

  layout->addWidget(select_destination_button, 1, 0, 1, 2);
  layout->addWidget(new QWidget(this), 1, 2, 1, 5);
  layout->addWidget(send_request_button, 1, 7, 1, 2);

  layout->addWidget(new QLabel("x:", this), 2, 0, 1, 1);
  layout->addWidget(destination_x_display, 2, 1, 1, 2);
  layout->addWidget(new QLabel("y:", this), 2, 3, 1, 1);
  layout->addWidget(destination_y_display, 2, 4, 1, 2);
  layout->addWidget(new QLabel("θ:", this), 2, 6, 1, 1);
  layout->addWidget(destination_yaw_display, 2, 7, 1, 2);

  QWidget* subgroup_widget = new QWidget(this);
  subgroup_widget->setLayout(layout);
  request_group_layout->addWidget(subgroup_widget);

  connect(select_destination_button, &QPushButton::clicked, this,
      &FreeFleetPanel::select_robot_destination);
  connect(send_request_button, &QPushButton::clicked, this,
      &FreeFleetPanel::send_robot_destination_request);
}

//=============================================================================

void FreeFleetPanel::create_path_request_subgroup()
{
  QLabel* path_subgroup_label = new QLabel(tr("Path:"), this);

  QPushButton* select_path_button = new QPushButton("Select", this);
  QPushButton* send_request_button = new QPushButton(tr("Send"), this);

  // setup the scroll area
  path_display = new QLabel("", this);
  QScrollArea* path_display_scroll_area = new QScrollArea(this);
  path_display_scroll_area->setWidget(path_display);

  QGridLayout* layout = new QGridLayout(this);
  layout->addWidget(path_subgroup_label, 0, 0, 1, 1);

  layout->addWidget(select_path_button, 1, 0, 1, 1);
  layout->addWidget(new QWidget(this), 1, 1, 1, 4);
  layout->addWidget(send_request_button, 1, 5, 1, 1);

  layout->addWidget(path_display_scroll_area, 2, 0, 1, 6);

  // add widget for scroll area
  
  QWidget* subgroup_widget = new QWidget(this);
  subgroup_widget->setLayout(layout);
  request_group_layout->addWidget(subgroup_widget);

  connect(select_path_button, &QPushButton::clicked, this,
      &FreeFleetPanel::select_robot_path);
  connect(send_request_button, &QPushButton::clicked, this,
      &FreeFleetPanel::send_robot_path_request);
}

//=============================================================================

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

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rs.location.yaw, Eigen::Vector3d::UnitZ());
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    array.markers.push_back(marker);
  }
  marker_array_pub->publish(array);
}

//=============================================================================

void FreeFleetPanel::refresh_values()
{
  // Update the number of robots registered
  number_of_robots_display->setText(
      QString(std::to_string(robot_states.size()).c_str()));

  // Update the robot mode of the current selected robot
  display_robot_mode();
}

//=============================================================================

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

//=============================================================================

} // namespace viz
} // namespace free_fleet

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(free_fleet::viz::FreeFleetPanel, rviz_common::Panel)
