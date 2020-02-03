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

#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>

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
#include <thread>
#include <unordered_map>

namespace free_fleet
{
namespace viz
{

class FreeFleetPanel : public rviz_common::Panel
{

Q_OBJECT

public:

  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  using RobotMode = rmf_fleet_msgs::msg::RobotMode;
  using RobotState = rmf_fleet_msgs::msg::RobotState;
  using FleetState = rmf_fleet_msgs::msg::FleetState;

  using ModeRequest = rmf_fleet_msgs::msg::ModeRequest;
  using PathRequest = rmf_fleet_msgs::msg::PathRequest;
  using DestinationRequest = rmf_fleet_msgs::msg::DestinationRequest;

  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  using ColorRGBA = std_msgs::msg::ColorRGBA;
  using Point = geometry_msgs::msg::Point;
  using Vector3 = geometry_msgs::msg::Vector3;
  using Pose = geometry_msgs::msg::Pose;

  FreeFleetPanel(QWidget* parent = 0);

  ~FreeFleetPanel();

private Q_SLOTS:

  void refresh_fleet_name();

  void refresh_robot_name_list();

  void move_to_robot();

  void follow_robot();

  void send_robot_mode_request();

  void select_robot_destination();

  void send_robot_destination_request();

  void select_robot_path();

  void send_robot_path_request();

private:

  //===========================================================================
  /// Fleet name components

  void create_fleet_name_group();
  
  QGroupBox* fleet_name_group_box;

  QLineEdit* fleet_name_editor;

  std::mutex fleet_name_mutex;

  QString fleet_name;

  QLabel* number_of_robots_display;

  //===========================================================================
  /// Robot components

  void create_robot_name_group();

  QGroupBox* robot_name_group_box;

  QComboBox* robot_name_combo;

  QString robot_name_placeholder;

  QString robot_mode_placeholder;

  QLabel* current_robot_mode;

  std::mutex robot_states_mutex;
  
  std::unordered_map<std::string, RobotState> robot_states;

  void clear_robot_states();

  void selected_robot_name(const QString& robot_name);

  void display_robot_mode();

  //===========================================================================
  /// Request group components

  void create_request_group();

  QGroupBox* request_group_box;

  QVBoxLayout* request_group_layout;

  //===========================================================================
  /// Mode Request subgroup components

  void create_mode_request_subgroup();

  QComboBox* mode_selection_combo;

  //===========================================================================
  /// Destination Request subgroup components

  void create_destination_request_subgroup();

  QGroupBox* destination_request_group_box;

  QString coordinate_placeholder;

  double destination_x;

  double destination_y;

  double destination_yaw;

  QLabel* destination_x_display;

  QLabel* destination_y_display;

  QLabel* destination_yaw_display;

  //===========================================================================
  /// Path Request subgroup components

  void create_path_request_subgroup();

  QGroupBox* path_request_group_box;

  QLabel* path_display;

  //============================================================================
  /// Request utilities

  std::string generate_new_random_task_id(unsigned int num_chars = 20) const;

  //===========================================================================
  /// ROS components

  void initialize_ros();

  rclcpp::Node::SharedPtr ros_node;

  rclcpp::Subscription<FleetState>::SharedPtr fleet_state_sub;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_array_pub;

  rclcpp::Publisher<ModeRequest>::SharedPtr mode_request_pub;

  rclcpp::Publisher<PathRequest>::SharedPtr path_request_pub;

  rclcpp::Publisher<DestinationRequest>::SharedPtr destination_request_pub;

  rclcpp::TimerBase::SharedPtr refresh_values_timer;

  void fleet_state_cb_fn(FleetState::UniquePtr msg);

  std::thread ros_thread;

  void ros_thread_fn();

  void refresh_values();

  Marker get_robot_marker_with_shape();

};

} // namespace viz
} // namespace free_fleet

#endif // FREE_FLEET_RVIZ2_PLUGIN__SRC__FREEFLEETPANEL_HPP
