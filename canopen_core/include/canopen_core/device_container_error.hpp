#ifndef DEVICE_CONTAINER_ERROR_HPP_
#define DEVICE_CONTAINER_ERROR_HPP_

#include <system_error>
#include <string>

namespace ros2_canopen
{

  class DeviceContainerException : public std::exception
  {
  private:
    std::string what_;
  public:
    DeviceContainerException(std::string what)
    {
      what_ = what;
    }

    char *what();
  };

}

#endif