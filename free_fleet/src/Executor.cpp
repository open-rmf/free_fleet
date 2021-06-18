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

#include <atomic>
#include <thread>
#include <functional>

#include <free_fleet/Console.hpp>
#include <free_fleet/Executor.hpp>

namespace free_fleet {

//==============================================================================
class Executor::Implementation
{
public:

  Implementation()
  {}

  Implementation(const Implementation&)
  {
    // This is only used during the construction of the implementation class.
  }

  ~Implementation()
  {
    stopped = true;
    if (async_thread.joinable())
      async_thread.join();
  }

  void run(std::chrono::nanoseconds period)
  {
    auto prev_t = std::chrono::steady_clock::now();
    while(!stopped.load() && worker)
    {
      prev_t = std::chrono::steady_clock::now();
      worker->run_once();
      std::this_thread::sleep_until(prev_t + period); 
    } 
  }

  void start_async(std::chrono::nanoseconds period)
  {
    async_thread =
      std::thread(std::bind(&Executor::Implementation::run, this, period));
  }

  std::unique_ptr<Worker> worker;

  std::thread async_thread;

  std::atomic<bool> stopped = true;
};

//==============================================================================
Executor::Executor(std::unique_ptr<Worker> worker)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  _pimpl->worker = std::move(worker);
}

//==============================================================================
Worker* Executor::worker()
{
  return _pimpl->worker.get();
}

//==============================================================================
void Executor::run(std::chrono::nanoseconds period)
{
  if (started())
  {
    fferr << "Executor has already been started.\n";
    throw std::runtime_error("Executor has already been started.");
  }

  _pimpl->stopped = false;
  _pimpl->run(period);
}

//==============================================================================
void Executor::start_async(std::chrono::nanoseconds period)
{
  if (started())
  {
    fferr << "Executor has already been started.\n";
    throw std::runtime_error("Executor has already been started.");
  }

  _pimpl->stopped = false;
  _pimpl->start_async(period);
}

//==============================================================================
bool Executor::started() const
{
 return !_pimpl->stopped.load();
}

//==============================================================================
void Executor::stop()
{
  _pimpl->stopped = true;
  if (_pimpl->async_thread.joinable())
    _pimpl->async_thread.join();
}

//==============================================================================
} // namespace free_fleet
