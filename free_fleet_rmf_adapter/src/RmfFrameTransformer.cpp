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

#include <eigen3/Geometry>

#include "RmfFrameTransformer.hpp"

namespace free_fleet {

//==============================================================================

RmfFrameTransformer::SharedPtr RmfFrameTransformer::make(
    Transformation transformation)
{
  SharedPtr transformer_ptr = std::make_shared(new RmfFrameTransformer);
  transformer_ptr->_config(std::move(transformation));
  return transformer_ptr;
}

//==============================================================================

void RmfFrameTransformer::transform_rmf_to_fleet(
    const messages::Location& rmf_frame_location,
    messages::Location& fleet_frame_location) const
{
  // It feels easier to read if each operation is a separate statement.
  // The compiler will be super smart and elide all these operations.
  const Eigen::Vector2d scaled = 
      _config.scale *
      Eigen::Vector2d(rmf_frame_location.x, rmf_frame_location.y);

  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet scaled: (%.3f, %.3f)",
  //     scaled[0], scaled[1]);

  const Eigen::Vector2d rotated =
      Eigen::Rotation2D<double>(_config.rotation) * scaled;
  
  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet rotated: (%.3f, %.3f)",
  //     rotated[0], rotated[1]);

  const Eigen::Vector2d translated =
      rotated + Eigen::Vector2d(_config.translation_x, _config.translation_y);

  // RCLCPP_INFO(
  //     get_logger(), "    rmf->fleet translated: (%.3f, %.3f)",
  //     translated[0], translated[1]);

  fleet_frame_location.x = translated[0];
  fleet_frame_location.y = translated[1];
  fleet_frame_location.yaw = rmf_frame_location.yaw + _config.rotation;

  fleet_frame_location.t = rmf_frame_location.t;
  fleet_frame_location.level_name = rmf_frame_location.level_name;
}

//==============================================================================

void RmfFrameTransformer::transform_fleet_to_rmf(
    const messages::Location& fleet_frame_location,
    messages::Location& rmf_frame_location) const
{
  // It feels easier to read if each operation is a separate statement.
  // The compiler will be super smart and elide all these operations.
  const Eigen::Vector2d translated =
      Eigen::Vector2d(fleet_frame_location.x, fleet_frame_location.y)
      - Eigen::Vector2d(_config.translation_x, _config.translation_y);

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf translated: (%.3f, %.3f)",
  //     translated[0], translated[1]);

  const Eigen::Vector2d rotated =
      Eigen::Rotation2D<double>(-_config.rotation) * translated;

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf rotated: (%.3f, %.3f)",
  //     rotated[0], rotated[1]);

  const Eigen::Vector2d scaled = 1.0 / _config.scale * rotated;

  // RCLCPP_INFO(
  //     get_logger(), "    fleet->rmf scaled: (%.3f, %.3f)",
  //     scaled[0], scaled[1]);
      
  rmf_frame_location.x = scaled[0];
  rmf_frame_location.y = scaled[1];
  rmf_frame_location.yaw = fleet_frame_location.yaw - _config.rotation;

  rmf_frame_location.t = fleet_frame_location.t;
  rmf_frame_location.level_name = fleet_frame_location.level_name;
}

//==============================================================================

RmfFrameTransformer::RmfFrameTransformer()
{}

//==============================================================================

} // namespace free_fleet
