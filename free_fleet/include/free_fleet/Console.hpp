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

#ifndef INCLUDE__FREE_FLEET__CONSOLE_HPP
#define INCLUDE__FREE_FLEET__CONSOLE_HPP

#include <string>
#include <memory>
#include <iostream>

#include <rmf_utils/impl_ptr.hpp>

namespace free_fleet {

/// \brief Output a message
#define ffinfo (free_fleet::Console::instance()->stream( \
  "Info", __FILE__, __FUNCTION__, __LINE__, \
  free_fleet::Console::StreamType::Info))

/// \brief Output a warning message
#define ffwarn (free_fleet::Console::instance()->stream( \
  "Warning", __FILE__, __FUNCTION__, __LINE__, \
  free_fleet::Console::StreamType::Warning))

/// \brief Output an error message
#define fferr (free_fleet::Console::instance()->stream( \
  "Error", __FILE__, __FUNCTION__, __LINE__, \
  free_fleet::Console::StreamType::Error))

/// \brief Console class that handles console outputs.
class Console 
{
public:

  /// \brief Stream class that is used by Console.
  class ConsoleStream
  {
  public:
    /// \brief Constructor.
    /// \param[in] stream Pointer to an output stream. Can be nullptr.
    ConsoleStream(std::ostream* stream);

    /// \brief Get the current output stream.
    /// \return Pointer to the current output stream.
    std::ostream* get_stream();
    
    /// \brief Redirects what is passed in to the stream object.
    /// \param[in] rhs Content to be logged.
    /// \return Reference to this ConsoleStream.
    template <class T>
    ConsoleStream& operator<<(const T& rhs);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  }; 

  /// \brief Types of stream outputs.
  enum class StreamType
  {
    Info,
    Warning,
    Error
  };

  /// \brief Retuns an instance to the Console class.
  static std::shared_ptr<Console> instance();

  /// \brief Deletes the current static Console instance to make way for a new
  /// one.
  static void clear();

  /// \brief Use this to obtain a stream to the console.
  /// \param[in] label The text label that will be printed before the message.
  /// \param[in] file The file name.
  /// \param[in] function The function name.
  /// \param[in] line The line number.
  /// \param[in] type The stream type of this output.
  /// \return Reference to the ConsoleStream.
  ConsoleStream& stream(
    const std::string& label, const std::string& file,
    const std::string& function, unsigned int line, StreamType type);

  class Implementation;
private:
  Console();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//=============================================================================
template<class T>
Console::ConsoleStream& Console::ConsoleStream::operator<<(const T& rhs)
{
  if (get_stream())
    *get_stream() << rhs;
  else
    std::cout << rhs;

  return *this;
}

//=============================================================================
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET__CONSOLE_HPP
