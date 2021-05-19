// Copyright (c) 2016-2019, Fraunhofer, Mathias Lüdtke, AutonomouStuff
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef SOCKETCAN_INTERFACE__LOGGING_HPP_
#define SOCKETCAN_INTERFACE__LOGGING_HPP_

#include <console_bridge/console.h>

#include <sstream>
#include <string>

#define ROSCANOPEN_LOG(name, file, line, level, args) \
  { \
    std::stringstream sstr; \
    sstr << name << ": " << args; \
    console_bridge::getOutputHandler()->log(sstr.str(), level, file, line); \
  }

#define ROSCANOPEN_ERROR(name, args) ROSCANOPEN_LOG( \
    name, __FILE__, __LINE__, \
    console_bridge::CONSOLE_BRIDGE_LOG_ERROR, args)
#define ROSCANOPEN_INFO(name, args) ROSCANOPEN_LOG( \
    name, __FILE__, __LINE__, \
    console_bridge::CONSOLE_BRIDGE_LOG_INFO, args)
#define ROSCANOPEN_WARN(name, args) ROSCANOPEN_LOG( \
    name, __FILE__, __LINE__, \
    console_bridge::CONSOLE_BRIDGE_LOG_WARN, args)
#define ROSCANOPEN_DEBUG(name, args) ROSCANOPEN_LOG( \
    name, __FILE__, __LINE__, \
    console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, args)

// extra function to mark it as deprecated
inline __attribute__ ((deprecated("please use ROSCANOPEN_* macros")))
void roscanopen_log_deprecated(const std::string s, const char * f, int l)
{
  console_bridge::getOutputHandler()->log(s, console_bridge::CONSOLE_BRIDGE_LOG_ERROR, f, l);
}

#define LOG(args) \
  { \
    std::stringstream sstr; sstr << "LOG: " << args; \
    roscanopen_log_deprecated(sstr.str(), __FILE__, __LINE__); \
  }

#endif  // SOCKETCAN_INTERFACE__LOGGING_HPP_
