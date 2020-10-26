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

#include <Eigen/Geometry>

#include <free_fleet/FrameTransformer.hpp>

namespace free_fleet {

//==============================================================================
class FrameTransformer::Implementation
{
public:

  Implementation()
  {}

  double _scale;
  double _rotation_yaw;
  Eigen::Vector2d _translation;
};

//==============================================================================
FrameTransformer::SharedPtr FrameTransformer::make(
  double scale,
  double translation_x,
  double translation_y,
  double rotation_yaw)
{
  SharedPtr frame_transformer(new FrameTransformer);
  frame_transformer->_pimpl->_scale = scale;
  frame_transformer->_pimpl->_rotation_yaw = rotation_yaw;
  frame_transformer->_pimpl->_translation = {translation_x, translation_y};
  return frame_transformer;
}

//==============================================================================
FrameTransformer::FrameTransformer()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
void FrameTransformer::forward_transform(
  const messages::Location& input,
  messages::Location& output)
{
  const Eigen::Vector2d scaled =
    _pimpl->_scale * Eigen::Vector2d(input.x, input.y);
  const Eigen::Vector2d rotated =
    Eigen::Rotation2D<double>(_pimpl->_rotation_yaw) * scaled;
  const Eigen::Vector2d translated = rotated + _pimpl->_translation;
  
  output = messages::Location{
    input.sec,
    input.nanosec,
    translated[0],
    translated[1],
    input.yaw + _pimpl->_rotation_yaw,
    input.level_name};
}

//==============================================================================
void FrameTransformer::backward_transform(
  const messages::Location& input,
  messages::Location& output)
{
  const Eigen::Vector2d translated =
    Eigen::Vector2d(input.x, input.y) - _pimpl->_translation;
  const Eigen::Vector2d rotated =
    Eigen::Rotation2D<double>(-_pimpl->_rotation_yaw) * translated;
  const Eigen::Vector2d scaled = 1.0 / _pimpl->_scale * rotated;

  output = messages::Location{
    input.sec,
    input.nanosec,
    scaled[0],
    scaled[1],
    input.yaw - _pimpl->_rotation_yaw,
    input.level_name};
}

//==============================================================================
} // namespace free_fleet
