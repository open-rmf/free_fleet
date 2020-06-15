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

#ifndef FF_RMF_ADAPTER__SRC__ROBOTCOMMAND_HPP
#define FF_RMF_ADAPTER__SRC__ROBOTCOMMAND_HPP

#include <rclcpp/rclcpp.hpp>

#include <free_fleet/RequestPublisher.hpp>

// #include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>

#include "RmfFrameTransformer.hpp"

namespace free_fleet {

class RobotCommand : public rmf_fleet_adapter::agv::RobotCommandHandle
{

public:

  struct Config
  {
    int domain_id = 42;
    std::string mode_request_topic = "mode_request";
    std::string path_request_topic = "path_request";
    std::string destination_request_topic = "destination_request";

    std::string fleet_name = "fleet_name";
    std::string robot_name = "robot_name";

    // the transformation order of operations from the adapter to the clients
    // 1) scale
    // 2) rotate
    // 3) translate
    double scale = 1.0;
    double rotation = 0.0;
    double translation_x = 0.0;
    double translation_y = 0.0;

    void print_config() const;

    RequestPublisher::Config request_publisher_config() const;
  };

  using SharedPtr = std::shared_ptr<RobotCommand>;

  static SharedPtr make(std::shared_ptr<rclcpp::Node> node, Config config);

  void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      ArrivalEstimator next_arrival_estimator,
      std::function<void()> path_finished_callback) final;

  void stop() final;

  /// Free fleet robots are by default without any docking procedure. This
  /// function is implemented as a no-op.
  void dock(
      const std::string& dock_name,
      std::function<void()> docking_finished_callback) final;

private:

  bool _active;
  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::TimerBase::SharedPtr _timer;

  Config _config;
  RequestPublisher::SharedPtr _request_publisher;

  RmfFrameTransformer::SharedPtr _frame_transformer;

  RobotCommand(
      std::shared_ptr<rclcpp::Node> node,
      RequestPublisher::SharedPtr request_publisher,
      RmfFrameTransformer::SharedPtr frame_transformer,
      Config config);

};

} // namespace free_fleet

#endif // FF_RMF_ADAPTER__SRC__ROBOTCOMMAND_HPP
