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

#ifndef TEST__UNIT__MOCK_CLIENTMIDDLEWARE_HPP
#define TEST__UNIT__MOCK_CLIENTMIDDLEWARE_HPP

#include <free_fleet/transport/ClientMiddleware.hpp>

namespace free_fleet {

class MockClientMiddleware : public transport::ClientMiddleware
{
public:

  MockClientMiddleware()
  {}

  void send_state(const messages::RobotState&) final
  {}

  rmf_utils::optional<messages::ModeRequest> read_mode_request() final
  {
    return rmf_utils::nullopt;
  }

  rmf_utils::optional<messages::NavigationRequest>
    read_navigation_request() final
  {
    return rmf_utils::nullopt;
  }

  rmf_utils::optional<messages::RelocalizationRequest>
    read_relocalization_request() final
  {
    return rmf_utils::nullopt;
  }

  messages::ModeRequest _prev_mode_request;
  messages::NavigationRequest _prev_nav_request;
  messages::RelocalizationRequest _prev_reloc_request;
};

} // namespace free_fleet

#endif // TEST__UNIT__MOCK_CLIENTMIDDLEWARE_HPP
