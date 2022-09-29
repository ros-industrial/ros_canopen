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

  class MasterException : public std::exception
  {
  private:
    std::string where_;
    MasterErrorCode code_;

  public:
    MasterException(MasterErrorCode code, std::string where)
    {
      where_ = where;
      code_ = code;
    }

    char *what();
  };
}

#endif