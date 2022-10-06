#ifndef DRIVER_ERROR_HPP_
#define DRIVER_ERROR_HPP_

#include <system_error>
#include <string>

namespace ros2_canopen
{

  enum DriverErrorCode
  {
    DriverNotMasterSet = 1,
    DriverNotInitialised = 2,
    DriverNotConfigured = 3,
    DriverNotActivated = 4,
    DriverAlreadyMasterSet = 5,
    DriverAlreadyInitialised = 6,
    DriverAlreadyConfigured = 7,
    DriverAlreadyActivated = 8,
    DriverFailedAddingToMaster = 9,
    DriverFailedRemovnigFromMaster = 10,
    DriverManual = 11,
  };

  class DriverException : public std::exception
  {
  private:
    std::string where_;
    DriverErrorCode code_;
  public:
    DriverException(DriverErrorCode code, std::string where)
    {
      where_ = where;
      code_ = code;
    }

    char *what();
  };

}

#endif