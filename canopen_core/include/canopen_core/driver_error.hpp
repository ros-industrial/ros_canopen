#ifndef DRIVER_ERROR_HPP_
#define DRIVER_ERROR_HPP_

#include <string>
#include <system_error>

namespace ros2_canopen
{
/**
 * @brief Driver Exception
 *
 * This exception is used, when a driver
 * fails.
 *
 */
class DriverException : public std::exception
{
private:
  std::string what_;

public:
  DriverException(std::string what) { what_ = what; }

  char * what();
};

}  // namespace ros2_canopen

#endif
