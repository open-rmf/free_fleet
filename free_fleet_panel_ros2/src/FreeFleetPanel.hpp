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

#ifndef FREE_FLEET_RVIZ2_PLUGIN__SRC__FREEFLEETPANEL_HPP
#define FREE_FLEET_RVIZ2_PLUGIN__SRC__FREEFLEETPANEL_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <rviz_common/panel.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <QLabel>
#include <QString>
#include <QGroupBox>
#include <QComboBox>
#include <QLineEdit>
#include <QVBoxLayout>

#include <mutex>
#include <unordered_map>

namespace free_fleet
{
namespace viz
{

class FreeFleetPanel : public rviz_common::Panel, public rclcpp::Node
{

Q_OBJECT

public:

  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  using RobotState = rmf_fleet_msgs::msg::RobotState;
  using FleetState = rmf_fleet_msgs::msg::FleetState;

  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  using ColorRGBA = std_msgs::msg::ColorRGBA;
  using Point = geometry_msgs::msg::Point;
  using Vector3 = geometry_msgs::msg::Vector3;
  using Pose = geometry_msgs::msg::Pose;

  FreeFleetPanel(QWidget* parent = 0);

public Q_SLOTS:

  void set_fleet_name(const QString& fleet_name);

protected Q_SLOTS:

  void update_fleet_name();

private:

  /// Fleet name group
  QGroupBox* fleet_name_group_box;
  QLineEdit* fleet_name_editor;

  /// Robot name, move-to and follow group
  QGroupBox* robot_name_group_box;
  QComboBox* robot_name_combo;
  QString default_robot_name_selection;

  /// Robot request group
  QGroupBox* request_group_box;
  QVBoxLayout* request_group_layout;

  /// Robot mode request subgroup
  QComboBox* mode_selection_combo;

  /// Robot destination request subgroup
  QGroupBox* destination_request_group_box;
  double destination_x;
  double destination_y;
  double destination_yaw;
  QLabel* destination_x_display;
  QLabel* destination_y_display;
  QLabel* destination_yaw_display;

  /// Robot path request subgroup
  QGroupBox* path_request_group_box;
  QLabel* path_display;

  // ---------------------------------------------------------------------------
  /// Group creation helper functions

  void create_fleet_name_group();

  void create_robot_name_group();

  void create_request_group();

  void create_mode_request_subgroup();

  void create_destination_request_subgroup();

  void create_path_request_subgroup();

  // ---------------------------------------------------------------------------
  /// Internal states

  std::mutex fleet_name_mutex;
  QString fleet_name;

  std::mutex robot_states_mutex;
  std::unordered_map<std::string, RobotState> robot_states;

  // ---------------------------------------------------------------------------
  /// ROS components

  rclcpp::Subscription<FleetState>::SharedPtr fleet_state_sub;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_array_pub;

  void fleet_state_cb_fn(FleetState::UniquePtr msg);

  void publish_marker_array();

};

} // namespace viz
} // namespace free_fleet

#endif // FREE_FLEET_RVIZ2_PLUGIN__SRC__FREEFLEETPANEL_HPP
