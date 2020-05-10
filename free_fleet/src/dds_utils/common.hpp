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

#ifndef FREEFLEET__SRC__DDS_UTILS__COMMON_HPP
#define FREEFLEET__SRC__DDS_UTILS__COMMON_HPP

#include <string>

namespace free_fleet {
namespace common {

char* dds_string_alloc_and_copy(const std::string& str);

} // namespace common
} // namespace free_fleet

#endif // FREEFLEET__SRC__DDS_UTILS__COMMON_HPP
