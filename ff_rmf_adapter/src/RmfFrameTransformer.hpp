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

#ifndef FF_RMF_ADAPTER__SRC__RMFFRAMETRANSFORMER_HPP
#define FF_RMF_ADAPTER__SRC__RMFFRAMETRANSFORMER_HPP

#include <messages/Location.hpp>

namespace free_fleet {

class RmfFrameTransformer
{
public:

  struct Transformation
  {
    double scale = 1.0;
    double rotation = 0.0;
    double translation_x = 0.0;
    double translation_y = 0.0;
  };

  using SharedPtr = std::shared_ptr<RmfFrameTransformer>;

  static SharedPtr make(Transformation transformation);

  void transform_rmf_to_fleet(
      const messages::Location& rmf_frame_location,
      messages::Location& fleet_frame_location) const;

  void transform_fleet_to_rmf(
      const messages::Location& fleet_frame_location,
      messages::Location& rmf_frame_location) const;

private:

  Transformation _config;

  RmfFrameTransformer();
};

} // namespace free_fleet

#endif // FF_RMF_ADAPTER__SRC__RMFFRAMETRANSFORMER_HPP
