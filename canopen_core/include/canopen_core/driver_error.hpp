#ifndef DRIVER_ERROR_HPP_
#define DRIVER_ERROR_HPP_

#include <system_error>
#include <string>

namespace ros2_canopen
{

  class DriverException : public std::exception
  {
  private:
    std::string what_;
  public:
    DriverException(std::string what)
    {
      what_ = what;
    }

    char *what();
  };

}

#endif