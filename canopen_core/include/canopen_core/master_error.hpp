#ifndef MASTER_ERROR_HPP_
#define MASTER_ERROR_HPP_

#include <system_error>

namespace ros2_canopen
{

  enum MasterErrorCode
  {
    MasterNotMasterSet = 1,
    MasterNotInitialised = 2,
    MasterNotConfigured = 3,
    MasterNotActivated = 4,
    MasterAlreadyMasterSet = 5,
    MasterAlreadyInitialised = 6,
    MasterAlreadyConfigured = 7,
    MasterAlreadyActivated = 8,
  };

  struct MasterErrorCategory : std::error_category
  {
    const char *name() const noexcept override;
    std::string message(int ev) const override;
  };
}

namespace std
{
  template <>
  struct is_error_code_enum<ros2_canopen::MasterErrorCategory> : true_type
  {
  };
  std::error_code make_error_code(ros2_canopen::MasterErrorCode);
}

#endif