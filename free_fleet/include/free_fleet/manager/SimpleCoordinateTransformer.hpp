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

#ifndef INCLUDE__FREE_FLEET__MANAGER__SIMPLECOORDINATETRANSFORMER_HPP
#define INCLUDE__FREE_FLEET__MANAGER__SIMPLECOORDINATETRANSFORMER_HPP

#include <memory>
#include <rmf_utils/impl_ptr.hpp>
#include <free_fleet/manager/CoordinateTransformer.hpp>

namespace free_fleet {
namespace manager {

class SimpleCoordinateTransformer : public CoordinateTransformer
{
public:

  /// Factory function that creates an instance of the Simple Coordinate
  /// Transformer, which handles scale, translation in x, y, and yaw rotations.
  static std::shared_ptr<SimpleCoordinateTransformer> make(
    double scale,
    double translation_x,
    double translation_y,
    double rotation_yaw);

  messages::Location forward_transform(const messages::Location& input)
    const final;

  messages::Location backward_transform(const messages::Location& input)
    const final;

  class Implementation;
private:
  SimpleCoordinateTransformer();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace manager
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__MANAGER__SIMPLECOORDINATETRANSFORMER_HPP
