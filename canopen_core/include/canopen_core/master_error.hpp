#ifndef MASTER_ERROR_HPP_
#define MASTER_ERROR_HPP_

#include <system_error>

namespace ros2_canopen
{

  class MasterException : public std::exception
  {
  private:
    std::string what_;

  public:
    MasterException(std::string what)
    {
      what_ = what;
    }

    char *what();
  };
}

#endif