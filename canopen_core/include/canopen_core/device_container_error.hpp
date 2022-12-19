#ifndef DEVICE_CONTAINER_ERROR_HPP_
#define DEVICE_CONTAINER_ERROR_HPP_

#include <string>
#include <system_error>

namespace ros2_canopen
{

/**
 * @brief Device Container Exception
 *
 * This exception is used, when the device container
 * fails.
 *
 */
class DeviceContainerException : public std::exception
{
private:
  std::string what_;

public:
  DeviceContainerException(std::string what) { what_ = what; }

  char * what();
};

}  // namespace ros2_canopen

#endif
