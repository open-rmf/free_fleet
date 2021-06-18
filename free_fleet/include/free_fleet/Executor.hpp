/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef INCLUDE__FREE_FLEET__EXECUTOR_HPP
#define INCLUDE__FREE_FLEET__EXECUTOR_HPP

#include <chrono>
#include <memory>

#include <free_fleet/Worker.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace free_fleet {

/// Generic executor that calls for task execution from its worker periodically,
/// when started.
class Executor 
{
public:
  
  /// Constructor.
  ///
  /// \param[in] worker
  ///   Unique worker that is handled by the executor.
  Executor(std::unique_ptr<Worker> worker);

  /// Get a pointer to the worker of this executor.
  ///
  /// \return Pointer to worker.
  Worker* worker();

  /// Starts the executor, if it has not yet been started. This function is
  /// blocking.
  ///
  /// \param[in] period
  ///   The interval used to routinely call the worker to perform its task.
  void run(std::chrono::nanoseconds period);

  /// Starts the executor, if it has not yet been started. This function is
  /// non-blocking.
  ///
  /// \param[in] period
  ///   The interval used to routinely call the worker to perform its task.
  void start_async(std::chrono::nanoseconds period);

  /// Checks if the executor has already been started.
  bool started() const;

  /// Stops the executor.
  void stop();

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namesace free_fleet

#endif // INCLUDE__FREE_FLEET__EXECUTOR_HPP
