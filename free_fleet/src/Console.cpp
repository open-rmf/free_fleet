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

#include <map>
#include <mutex>
#include <memory>
#include <sstream>

#include <free_fleet/Console.hpp>

namespace free_fleet {

//=============================================================================
// Static pointer to console.
static std::shared_ptr<Console> g_console; 
static std::mutex g_console_mutex;

//=============================================================================
class Console::ConsoleStream::Implementation
{
public:

  std::ostream* stream;
};

//=============================================================================
Console::ConsoleStream::ConsoleStream(std::ostream* stream)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{stream}))
{}

//=============================================================================
std::ostream* Console::ConsoleStream::get_stream()
{
  return _pimpl->stream;
}

//=============================================================================
class Console::Implementation
{
public:

  Implementation(std::ostream* info, std::ostream* warning, std::ostream* error)
  : info_stream(ConsoleStream(info)),
    warning_stream(ConsoleStream(warning)),
    error_stream(ConsoleStream(error))
  {}

  ConsoleStream info_stream;
  
  ConsoleStream warning_stream;

  ConsoleStream error_stream;
};

//=============================================================================
Console::Console()
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation(&std::cout, &std::cout, &std::cerr)))
{}

//=============================================================================
auto Console::instance() -> std::shared_ptr<Console>
{
  std::lock_guard<std::mutex> lock(g_console_mutex);
  if (!g_console)
  {
    g_console.reset(new Console());
  }
  return g_console;
}

//=============================================================================
void Console::clear()
{
  std::lock_guard<std::mutex> lock(g_console_mutex);
  g_console = nullptr;
}

//=============================================================================
Console::ConsoleStream& Console::stream(
  const std::string& label, const std::string& file,
  const std::string& function, unsigned int line, StreamType type)
{
  size_t index = file.find_last_of("/") + 1;

  std::string prefix = label + " [" + file.substr(index, file.size() - index)
    + ":" + function + ":" + std::to_string(line) + "] ";

  switch (type)
  {
    case StreamType::Info:
      return _pimpl->info_stream << "\033[0;32m" << prefix << "\033[0m";
    case StreamType::Warning:
      return _pimpl->warning_stream << "\033[1;33m" << prefix << "\033[0m";
    case StreamType::Error:
      return _pimpl->error_stream << "\033[0;31m" << prefix << "\033[0m";
    default:
      return _pimpl->info_stream << prefix;
  }
}

//=============================================================================
} // namespace free_fleet

