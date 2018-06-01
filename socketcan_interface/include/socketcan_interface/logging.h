#ifndef SOCKETCAN_INTERFACE_LOGGING_H
#define SOCKETCAN_INTERFACE_LOGGING_H

#include <console_bridge/console.h>
#include <sstream>

#define ROSCANOPEN_LOG(name, file, line, level, args) { std::stringstream sstr; sstr << name << ": " << args; console_bridge::getOutputHandler()->log(sstr.str(), level, file, line); }

#define ROSCANOPEN_ERROR(name, args) ROSCANOPEN_LOG(name, __FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_ERROR, args)
#define ROSCANOPEN_INFO(name, args) ROSCANOPEN_LOG(name, __FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_INFO, args)
#define ROSCANOPEN_WARN(name, args) ROSCANOPEN_LOG(name, __FILE__, __LINE__, console_bridge::CONSOLE_BRIDGE_LOG_WARN, args)
#define ROSCANOPEN_DEBUG(name, args) ROSCANOPEN_LOG(name, __FILE__, __LINE__,console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, args)

// extra function to mark it as deprecated
inline __attribute__ ((deprecated("please use ROSCANOPEN_* macros"))) void roscanopen_log_deprecated(const std::string s, const char* f, int l) { console_bridge::getOutputHandler()->log(s, console_bridge::CONSOLE_BRIDGE_LOG_ERROR, f, l); }
#define LOG(args) { std::stringstream sstr; sstr << "LOG: " << args; roscanopen_log_deprecated(sstr.str(), __FILE__, __LINE__); }
#endif
