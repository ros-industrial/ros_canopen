#ifndef DRIVER_ERROR_HPP_
#define DRIVER_ERROR_HPP_

#include <system_error>

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
  };

  struct DriverErrorCategory : std::error_category
  {
    const char *name() const noexcept override;
    std::string message(int ev) const override;
  };
}

namespace std
{
  template <>
  struct is_error_code_enum<ros2_canopen::DriverErrorCategory> : true_type
  {
  };
  std::error_code make_error_code(ros2_canopen::DriverErrorCode);
}

#endif