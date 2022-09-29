#include "canopen_core/driver_error.hpp"
#include <cstring>
namespace ros2_canopen
{

    char *DriverException::what()
    {
        std::string message;
        message.append(where_);
        switch (code_)
        {
        case DriverErrorCode::DriverNotMasterSet:
            message.append("Driver does not have a master object set.");
            break;
        case DriverErrorCode::DriverNotInitialised:
            message.append("Driver is not yet intialised.");
            break;
        case DriverErrorCode::DriverNotConfigured:
            message.append("Driver is not yet configured.");
            break;
        case DriverErrorCode::DriverNotActivated:
            message.append("Driver is not yet activated.");
            break;
        case DriverErrorCode::DriverAlreadyMasterSet:
            message.append("Driver does already have a master object set.");
            break;
        case DriverErrorCode::DriverAlreadyInitialised:
            message.append("Driver is already initialised.");
            break;
        case DriverErrorCode::DriverAlreadyConfigured:
            message.append("Driver is already configured.");
            break;
        case DriverErrorCode::DriverAlreadyActivated:
            message.append("Driver is already activated.");
            break;
        case DriverErrorCode::DriverFailedAddingToMaster:
            message.append("Driver could not be added to master.");
            break;
        case DriverErrorCode::DriverFailedRemovnigFromMaster:
            message.append("Driver could not be removed from master.");
            break;
        case DriverManual:
            break;
        default:
            message.append("(unrecognized error)");
            break;
        }
        
        char *res = new char[1000];  
        strcpy(res, message.c_str());
        return res;
    }
}