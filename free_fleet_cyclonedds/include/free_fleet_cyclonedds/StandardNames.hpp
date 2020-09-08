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

#ifndef SRC__STANDARDNAMES_HPP
#define SRC__STANDARDNAMES_HPP

#include <string>

namespace free_fleet {
namespace cyclonedds {

const std::string Prefix = "free_fleet/";
const std::string GraphTopicName = "navigation_graph";
const std::string StateTopicName = "robot_state";
const std::string ModeRequestTopicName = "mode_request";
const std::string NavigationRequestTopicName = "navigation_request";

} // namespace cyclonedds
} // namespace free_fleet


#endif // SRC__STANDARDNAMES_HPP
