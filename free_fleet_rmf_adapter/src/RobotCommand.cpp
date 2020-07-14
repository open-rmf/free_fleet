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

#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/ModeRequest.hpp>

#include "utilities.hpp"
#include "RobotCommand.hpp"

namespace free_fleet {

//==============================================================================

void RobotCommand::Config::print_config() const
{
  printf("ROBOT COMMAND CONFIGURATION\n");
  printf("  domain ID: %d\n", domain_id);
  printf("  fleet name: %s\n", fleet_name.c_str());
  printf("  robot name: %s\n", robot_name.c_str());
  printf("  TOPICS\n");
  printf("    mode request: %s\n", mode_request_topic.c_str());
  printf("    path request: %s\n", path_request_topic.c_str());
  printf("    destination request: %s\n", destination_request_topic.c_str());
  printf("COORDINATE TRANSFORMATION\n");
  printf("  translation x (meters): %.3f\n", translation_x);
  printf("  translation y (meters): %.3f\n", translation_y);
  printf("  rotation (radians): %.3f\n", rotation);
  printf("  scale: %.3f\n", scale);
}

//==============================================================================

RequestPublisher::Config RobotCommand::Config::request_publisher_config() const
{
  return RequestPublisher::Config {
    domain_id,
    mode_request_topic,
    path_request_topic,
    destination_request_topic
  };
}

//==============================================================================

RmfFrameTransformer::Transformation RobotCommand::Config::transformation() const
{
  return RmfFrameTransformer::Transformation {
    scale,
    rotation,
    translation_x,
    translation_y
  };
}

//==============================================================================

RobotCommand::SharedPtr RobotCommand::make(
    std::shared_ptr<rclcpp::Node> node,
    Config config,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits)
{
  RequestPublisher::SharedPtr request_publisher =
      RequestPublisher::make(config.request_publisher_config());
  if (!request_publisher)
    return nullptr;

  RmfFrameTransformer::SharedPtr frame_transformer =
      RmfFrameTransformer::make(config.transformation());
  if (!frame_transformer)
    return nullptr;

  RobotComamd::SharedPtr command_ptr(new RobotCommand);
  command_ptr->_active = false;
  command_ptr->_node = std::move(node);
  command_ptr->_request_publisher = std::move(request_publisher);
  command_ptr->_frame_transformer = std::move(frame_transformer);
  command_ptr->_travel_info.graph = std::move(graph);
  command_ptr->_travel_info.traits = std::move(traits);
  command_ptr->_travel_info.fleet_name = config.fleet_name;
  command_ptr->_travel_info.robot_name = config.robot_name;
  command_ptr->_config = std::move(config);
  return command_ptr;
}

//==============================================================================

void RobotCommand::follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    std::function<void()> path_finished_callback) final
{
  std::lock_guard<std::mutex> lock(_mutex);
  _clear_last_command();

  _travel_info.waypoints = waypoints;
  _travel_info.next_arrival_estimator = std::move(next_arrival_estimator);
  _travel_info.path_finished_callback = std::move(path_finished_callback);
  _interrupted = false;

  _current_path_request.task_id = std::to_string(++_current_task_id);
  _current_path_request.fleet_name = _travel_info.fleet_name;
  _current_path_request.robot_name = _travel_info.robot_name;
  _current_path_request.path.clear();
  for (const auto& wp : waypoints)
  {
    const Eigen::Vector3d pos = wp.position();
    messages::Location rmf_loc;
    rmf_loc.sec = rmf_traffic_ros2::convert(wp.time()).sec();
    rmf_loc.nanosec = rmf_traffic_ros2::convert(wp.time()).nanosec();
    rmf_loc.x = pos.x();
    rmf_loc.y = pos.y();
    rmf_loc.yaw = pos.z();
    if (wp.graph_index())
    {
      rmf_loc.level_name =
          _travel_info.graph->get_waypoint(*wp.graph_index()).get_map_name();
    }

    messages::Location ff_loc;
    _frame_transformer->transform_rmf_to_fleet(rmf_loc, fleet_loc);

    _current_path_request.path.emplace_back(std::move(fleet_loc));
  }
  
  if (!_request_publisher->send_path_request(_current_path_request))
  {
    RCLCPP_ERROR(
        _node->get_logger(), 
        "Failed to send path request [%s]",
        std::to_string(_current_task_id).c_str());
  }
}

//==============================================================================

void RobotCommand::stop() final
{
  messages::RobotMode mode_msg = { messages::RobotMode::MODE_PAUSED };
  _current_mode_request.fleet_name = _travel_info.fleet_name;
  _current_mode_request.robot_name = _travel_info.robot_name;
  _current_mode_request.mode = mode_msg;
  _current_mode_request.task_id = std::to_string(++_current_task_id);
  if (!_request_publisher->send_mode_request(_current_mode_request))
  {
    RCLCPP_ERROR(
        _node->get_logger(), 
        "Failed to send a stop request [%s]",
        std::to_string(_current_task_id).c_str());
  }
}

//==============================================================================

void RobotCommand::dock(
    const std::string& dock_name,
    std::function<void()> docking_finished_callback) final
{
  // Free fleet robots are by default without any docking procedure. This
  // function is implemented as a no-op.
}

//==============================================================================

void update_state(const messages::RobotState& state)
{
  std::lock_guard<std::mutex> lock(_mutex);
  if (_travel_info.path_finished_callback)
  {
    // If we have a path_finished_callback, then the robot should be
    // following a path

    // The arrival estimator should be available
    assert(_travel_info.next_arrival_estimator);

    if (state.task_id != _current_task_id)
    {
      if (_current_task_id == _current_path_request.task_id)
      {
        // The robot has not received our path request yet
        if (!_request_publisher->send_path_request(_current_path_request))
        {
          RCLCPP_ERROR(
              _node->get_logger(),
              "Failed to send path request [%s]",
              std::to_string(_current_task_id).c_str());
        }
      }
      else if (_current_task_id == _current_mode_request.task_id)
      {
        // The robot has not received our mode request yet
        if (!_request_publisher->send_mode_request(_current_mode_request))
        {
          RCLCPP_ERROR(
              _node->get_logger(),
              "Failed to send mode request [%s]",
              std::to_string(_current_task_id).c_str());
        }
      }

      return update_position(state);
    }

    if (state.mode.mode == state.mode.MODE_REQUEST_ERROR)
    {egardi
      if (_interrupted)
      {
        // This interruption was already noticed
        return;
      }

      RCLCPP_INFO(
            _node->get_logger(),
            "Fleet [%s] reported interruption for [%s]",
            _travel_info.fleet_name.c_str(),
            _travel_info.robot_name.c_str());
      _interrupted = true;
      update_position(state);
      return _travel_info.updater->interrupted();
    }

    if (state.path.empty())
    {
      // When the state path is empty, that means the robot believes it has
      // arrived at its destination.
      return update_position(state);
    }
  
    return update_position_with_path(state);
  }
  else if (_dock_finished_callback)
  {
    // TODO: docking
  }
  else
  {
    if (state.location.path.empty())
      update_position(state);
    else
      update_position_with_path(state);
  }
}

//==============================================================================

void RobotCommand::update_position(const messages::RobotState& state)
{
  // This fleet adapter will not be compliant and infer the map from
  // last known waypoints, level names have to be published in all states
  const messages::Location loc = state.location;
  if (loc.level_name.empty())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Robot named [%s] belonging to fleet [%s] is lost because we cannot "
          "figure out what floor it is on. Please publish the robot's current "
          "floor name in the level_name field of its RobotState.",
          _travel_info.robot_name.c_str(), _travel_info.fleet_name.c_str());
    return;
  }
 
  const rmf_traffic::agv::Graph::Waypoint* closest_wp = closest_waypoint(loc);
  assert(closest_wp);

  const Eigen::Vector2d p(loc.x, loc.y);
  const double nearest_dist = (p - closest_wp.get_location()).norm();
  
  // Really close enough to the waypoint, assume that was where it intended
  // to land on
  if (nearest_dist < 0.25)
  {
    _travel_info.last_known_wp = closest_wp->index();
    return _travel_info.updater->update_position(closest_wp->index(), loc.yaw);
  }
  // Not close enough, we will not assume that closest_wp was the last_known_wp,
  // update map and position as a last resort.
  return _travel_info.updater->update_position(
      loc.level_name, 
      {loc.x, loc.y, loc.yaw});
}

//==============================================================================

void RobotCommand::update_position_with_path(const messages::RobotState& state)
{
  // This fleet adapter will not be compliant and infer the map from
  // last known waypoints, level names have to be published in all states
  const messages::Location loc = state.location;
  if (loc.level_name.empty())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Robot named [%s] belonging to fleet [%s] is lost because we cannot "
          "figure out what floor it is on. Please publish the robot's current "
          "floor name in the level_name field of its RobotState.",
          _travel_info.robot_name.c_str(), _travel_info.fleet_name.c_str());
    return;
  }

  const rmf_traffic::agv::Graph::Waypoint* closest_wp = closest_waypoint(loc);
  assert(closest_wp);

  const Eigen::Vector2d p(loc.x, loc.y);
  const double nearest_dist = (p - closest_wp.get_location()).norm();

  if (nearest_dist < 0.25)
  {
    // Waypoint is near enough, to be updated with waypoint index and 
    // orientation
    _travel_info.last_known_wp = closest_wp->index();
    return _travel_info.updater->update_position(closest_wp->index(), loc.yaw);
  }
  else
  {
    // It is somewhere in between waypoints now, we will update with lanes
    assert(!state.path.empty());

    // Find nearest next waypoint using path's next waypoint
    const rmf_traffic::agv::Graph::Waypoint* closest_next_wp = 
        closest_waypoint(state.path[0]);
    assert(closest_next_wp);

    const Eigen::Vector2d p(state.path[0].x, state.path[0].y);
    const double nearest_dist = (p - closest_next_wp.get_location()).norm();

    // This next waypoint is close enough to the one in the graph
    if (nearest_dist < 0.25)
    {
      // Try to update position with lane between valid next waypoint and
      // last known wp
      if (_travel_info.last_known_wp)
      {
        std::vector<std::size_t> current_lanes;
        auto lanes = 
            _travel_info.graph->lanes_from(_travel_info.last_known_wp.index());
        for (std::size_t i : lanes)
        {
          std::size_t exit_wp_index = 
              _travel_info.graph->get_lane(i).exit().waypoint_index();
          if (exit_wp_index == closest_next_wp.index())
            current_lanes.push_back(i);
        }

        if (!current_lanes.empty())
        {
          // We found the lanes that it is travelling on, update with lane info
          return _travel_info.updater->update_position(
              {loc.x, loc.y, loc.yaw},
              current_lanes);
        }
      }

      // no last_known_wp or no valid lanes were found, update with target 
      // waypoint
      return _travel_info.updater->update_position(
          {loc.x, loc.y, loc.yaw},
          closest_next_wp->index());
    }
    // No valid next waypoint was found using the path from state, update
    // with map and position as last resort
    else
    {
      RCLCPP_ERROR(
          node->get_logger(),
          "Robot named [%s] belonging to fleet [%s] is lost because we cannot "
          "figure out where its next target waypoint is. The robot seems to be "
          "following a path that is unknown to RMF.",
          _travel_info.robot_name.c_str(), _travel_info.fleet_name.c_str());
      return _travel_info.updater->update_position(
          loc.level_name,
          {loc.x, loc.y, loc.yaw});
    }
  }
}

//==============================================================================

rmf_traffic::agv::Graph::Waypoint* RobotCommand::closest_waypoint(
    const messages::Location& location) const
{
  const std::string map_name = location.level_name;
  const Eigen::Vector2d p(loc.x, loc.y);
  const rmf_traffic::agv::Graph::Waypoint* closest_wp = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i=0; i < _travel_info.graph->num_waypoints(); ++i)
  {
    const auto& wp = _travel_info.graph->get_waypoint(i);
    if (map_name != wp.get_map_name())
      continue;

    const Eigen::Vector2d p_wp = wp.get_location();
    const double dist = (p -p_wp).norm();
    if (dist < nearest_dist)
    {
      closest_wp = &wp;
      nearest_dist = dist;
    }
  }
  return closest_wp;
}

//==============================================================================

RobotCommand::RobotCommand()
{}

//==============================================================================

} // namespace free_fleet
