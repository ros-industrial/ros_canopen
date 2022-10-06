#include "canopen_core/master_error.hpp"
#include <cstring>
namespace ros2_canopen
{

    char *MasterException::what()
    {
        std::string message;
        message.append(where_);

        switch (code_)
        {
        case MasterErrorCode::MasterNotMasterSet:
            message.append("Master does not have a master object set.");
            break;
        case MasterErrorCode::MasterNotInitialised:
            message.append("Master is not yet intialised.");
            break;
        case MasterErrorCode::MasterNotConfigured:
            message.append("Master is not yet configured.");
            break;
        case MasterErrorCode::MasterNotActivated:
            message.append("Master is not yet activated.");
            break;
        case MasterErrorCode::MasterAlreadyMasterSet:
            message.append("Master does already have a master object set.");
            break;
        case MasterErrorCode::MasterAlreadyInitialised:
            message.append("Master is already initialised.");
            break;
        case MasterErrorCode::MasterAlreadyConfigured:
            message.append("Master is already configured.");
            break;
        case MasterErrorCode::MasterAlreadyActivated:
            message.append("Master is already activated.");
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