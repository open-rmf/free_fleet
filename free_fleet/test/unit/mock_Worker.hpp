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

#ifndef TEST__UNIT__MOCK_WORKER_HPP
#define TEST__UNIT__MOCK_WORKER_HPP

#include <free_fleet/Console.hpp>
#include <free_fleet/Worker.hpp>

namespace free_fleet {

class MockWorker : public Worker
{
public:

  MockWorker(int worker_id, std::function<void()> run_once_fn)
  : _worker_id(worker_id),
    _run_once_fn(std::move(run_once_fn))
  {}

  void run_once() override
  {
    _run_once_fn();
  }

  int id() const
  {
    return _worker_id;
  }

  void id(int new_id)
  {
    _worker_id = new_id;
  } 

private:
  int _worker_id;

  std::function<void()> _run_once_fn;
};

} // namespace free_fleet

#endif // TEST__UNIT__MOCK_WORKER_HPP
