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

#ifndef INCLUDE__FREE_FLEET__MANAGER_HPP
#define INCLUDE__FREE_FLEET__MANAGER_HPP

#include <memory>
#include <vector>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <free_fleet/agv/RobotInfo.hpp>
#include <free_fleet/transport/Middleware.hpp>
#include <free_fleet/CoordinateTransformer.hpp>

#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/ModeParameter.hpp>

#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet/messages/RelocalizationRequest.hpp>

namespace free_fleet {

class Manager
{
public:

  using SharedPtr = std::shared_ptr<Manager>;

  /// This function returns the current time stamp based on the implementation.
  using TimeNow = std::function<rmf_traffic::Time()>;

  /// This callback function can be used to work with different applications,
  /// triggered every time a robot is updated with an incoming new state.
  using RobotUpdatedCallback =
    std::function<void(const std::shared_ptr<agv::RobotInfo>& robot_info)>;

  /// Factory function that creates an instance of the Free Fleet Manager.
  ///
  /// \param[in] fleet_name
  /// \param[in] graph
  /// \param[in] middleware
  /// \param[in] to_robot_transform
  /// \param[in] time_now_fn
  /// \param[in] robot_updated_callback_fn
  /// \return
  static SharedPtr make(
    const std::string& fleet_name,
    std::shared_ptr<rmf_traffic::agv::Graph> graph,
    std::shared_ptr<transport::Middleware> middleware,
    std::shared_ptr<CoordinateTransformer> to_robot_transform,
    TimeNow time_now_fn,
    RobotUpdatedCallback robot_updated_callback_fn);

  /// Starts the manager which begins to listen for clients, if it has not yet
  /// been started. This function is blocking.
  ///
  /// \param[in] frequency
  ///   Frequency at which the manager operates. This value needs to be a
  ///   non-zero value.
  void run(uint32_t frequency);

  /// Starts the manager which begins to listen for clients, if it has not yet
  /// been started. This function is non-blocking.
  ///
  /// \param[in] frequency
  ///   Frequency at which the manager operates. This value needs to be a
  ///   non-zero value.
  void start_async(uint32_t frequency);

  /// Checks if the manager has already been started.
  bool started() const;

  /// Gets all the names of the robots that are currently under this manager.
  ///
  /// \return
  std::vector<std::string> robot_names();

  /// Gets the RobotInfo of the robot with the provided name. If no such robot
  /// exists, a nullptr will be returned.
  ///
  /// \param[in] robot_name
  /// \return
  std::shared_ptr<agv::RobotInfo> robot(const std::string& robot_name);

  /// Gets all the available RobotInfo that has been registered with the manager
  ///
  /// \return
  std::vector<std::shared_ptr<agv::RobotInfo>> all_robots();

  /// Sends out a mode request to a robot.
  ///
  /// \param[in] robot_name
  ///   Name of robot that the request is targeted at.
  ///
  /// \param[in] mode
  ///   Desired robot mode.
  ///
  /// \param[in] parameters
  ///   Optional parameters for a mode request.
  ///
  /// \return
  ///   Optional of the task ID for this particular request. Returns a nullopt
  ///   if there does not exist a robot of the provided name.
  rmf_utils::optional<std::size_t> send_mode_request(
    const std::string& robot_name,
    const messages::RobotMode& mode,
    std::vector<messages::ModeParameter> parameters);

  /// Sends out a navigation request.
  ///
  /// \param[in] robot_name
  ///   Name of the robot that the request is targeted at.
  ///
  /// \param[in] path
  ///   Desired path of waypoints that the robot should follow.
  ///
  /// \return
  ///   Optional of the task ID for this particular request. Returns a nullopt
  ///   if there does not exist a robot of the provided name, if the provided
  ///   path is empty, or if any of the waypoints are non-conforming to the
  ///   navigation graph of the manager.
  rmf_utils::optional<std::size_t> send_navigation_request(
    const std::string& robot_name,
    const std::vector<messages::Waypoint>& path);

  /// Sends out a relocalization request.
  ///
  /// \param[in] robot_name
  ///   Name of the robot that the request is targeted at.
  ///
  /// \param[in] location
  ///   Desired relocalization location for the robot.
  ///
  /// \param[in] last_visited_waypoint_index
  ///   The last visited or nearest waypoint index on the navigation graph of
  ///   the robot, for it to continue tracking its progress through the graph.
  ///
  /// \return
  ///   Optional of the task ID for this particular request. Returns a nullopt
  ///   if there does not exist a robot of the provided name, if the last
  ///   visited waypoint index does not exist in the navigation graph, or if the
  ///   desired relocalization location is too far away from the last visited
  ///   waypoint.
  rmf_utils::optional<std::size_t> send_relocalization_request(
    const std::string& robot_name,
    const messages::Location& location,
    std::size_t last_visited_waypoint_index);

  class Implementation;
private:
  Manager();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MANAGER_HPP
