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

#include <iostream>
#include <Eigen/Geometry>
#include <free_fleet/Console.hpp>
#include <free_fleet/manager/SimpleCoordinateTransformer.hpp>

namespace free_fleet {
namespace manager {

//==============================================================================
class SimpleCoordinateTransformer::Implementation
{
public:

  double scale;
  double rotation_yaw;
  Eigen::Vector2d translation;
};

//==============================================================================
auto SimpleCoordinateTransformer::make(
  double scale,
  double translation_x,
  double translation_y,
  double rotation_yaw)
  -> std::shared_ptr<SimpleCoordinateTransformer>
{
  if (scale < 0)
  {
    fferr << "Provided scale is invalid, it has to be a double larger than "
      << "0.\n";
    return nullptr;
  }

  std::shared_ptr<SimpleCoordinateTransformer> transformer(
    new SimpleCoordinateTransformer);
  transformer->_pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      scale,
      rotation_yaw,
      {translation_x, translation_y}
    });
  return transformer;
}

//==============================================================================
SimpleCoordinateTransformer::SimpleCoordinateTransformer()
{}

//==============================================================================
messages::Location SimpleCoordinateTransformer::forward_transform(
  const messages::Location& input) const
{
  const Eigen::Vector2d scaled = _pimpl->scale * input.coordinates();
  const Eigen::Vector2d rotated =
    Eigen::Rotation2D<double>(_pimpl->rotation_yaw) * scaled;
  const Eigen::Vector2d translated = rotated + _pimpl->translation;

  if (input.yaw().has_value())
  {
    return messages::Location(
      input.map_name(), translated, input.yaw().value() + _pimpl->rotation_yaw);
  }

  return messages::Location(input.map_name(), translated);
}

//==============================================================================
messages::Location SimpleCoordinateTransformer::backward_transform(
  const messages::Location& input) const
{
  const Eigen::Vector2d translated = input.coordinates() - _pimpl->translation;
  const Eigen::Vector2d rotated =
    Eigen::Rotation2D<double>(-_pimpl->rotation_yaw) * translated;
  const Eigen::Vector2d scaled = 1.0 / _pimpl->scale * rotated;

  if (input.yaw().has_value())
  {
    return messages::Location(
      input.map_name(),
      scaled,
      input.yaw().value() - _pimpl->rotation_yaw);
  }

  return messages::Location(input.map_name(), scaled);
}

//==============================================================================
} // namespace manager
} // namespace free_fleet
