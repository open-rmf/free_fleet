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

#ifndef FREE_FLEET__INCLUDE__FREE_FLEET__REQUESTPUBLISHER_HPP
#define FREE_FLEET__INCLUDE__FREE_FLEET__REQUESTPUBLISHER_HPP

#include <string>
#include <memory>

#include <free_fleet/Participant.hpp>
#include <free_fleet/messages/ModeRequest.hpp>
#include <free_fleet/messages/PathRequest.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

namespace free_fleet {

class RequestPublisher
{

public:

  /// Configuration for setting up a RequestPublisher, defines the DDS domain,
  /// and various topic names for sending requests.
  struct Config
  {
    int domain_id = 42;
    std::string mode_request_topic = "mode_request";
    std::string path_request_topic = "path_request";
    std::string destination_request_topic = "destination_request";

    /// Prints the values of this RequestPublisher configuration. 
    void print_config() const;
  };

  using SharedPtr = std::shared_ptr<RequestPublisher>;

  /// Factory function that creates an instance of the Free Fleet Request 
  /// Publisher.
  ///
  /// \param[in] config
  ///   Configuration that sets up the request publisher.
  /// \param[in] participant
  ///   Shared pointer to a participant, that can be used to create the
  ///   subscriber, instead of re-initializing a participant.
  /// \return
  ///   Shared pointer to the request publisher, nullptr if creation failed.
  static SharedPtr make(
      Config config, Participant::SharedPtr participant = nullptr);

  /// Attempts to send a new mode request to all the clients. Clients are in
  /// charge to identify if requests are targetted towards them.
  /// 
  /// \param[in] mode_request
  ///   New mode request to be sent out to the clients.
  /// \return
  ///   True if the mode request was successfully sent, false otherwise.
  bool send_mode_request(const messages::ModeRequest& mode_request) const;

  /// Attempts to send a new path request to all the clients. Clients are in
  /// charge to identify if requests are targetted towards them.
  ///
  /// \param[in] path_request
  ///   New path request to be sent out to the clients.
  /// \return
  ///   True if the path request was successfully sent, false otherwise.
  bool send_path_request(const messages::PathRequest& path_request) const;

  /// Attempts to send a new destination request to all the clients. Clients 
  /// are in charge to identify if requests are targetted towards them.
  ///
  /// \param[in] destination_request
  ///   New destination request to be sent out to the clients.
  /// \return
  ///   True if the destination request was successfully sent, false otherwise.
  bool send_destination_request(
      const messages::DestinationRequest& destination_request) const;

  /// Destructor
  ~RequestPublisher();

  class Implementation;
private:
  RequestPublisher();
  std::unique_ptr<Implementation> _pimpl;  
};

} // free_fleet

#endif // FREE_FLEET__INCLUDE__FREE_FLEET__REQUESTPUBLISHER_HPP
