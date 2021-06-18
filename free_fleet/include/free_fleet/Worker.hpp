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

#ifndef INCLUDE__FREE_FLEET__WORKER_HPP
#define INCLUDE__FREE_FLEET__WORKER_HPP

namespace free_fleet {

/// Implement this class for a generic worker that works with an Executor.
class Worker
{
public:

  /// Have this worker run it's operation once.  
  virtual void run_once() = 0;
};

} // namesace free_fleet

#endif // INCLUDE__FREE_FLEET__WORKER_HPP
