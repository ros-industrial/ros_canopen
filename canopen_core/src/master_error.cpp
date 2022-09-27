#include "canopen_core/master_error.hpp"

const ros2_canopen::MasterErrorCategory MasterErrorCategoryInstance{};

std::error_code std::make_error_code(ros2_canopen::MasterErrorCode e)
{
    return {static_cast<int>(e), MasterErrorCategoryInstance};
}

namespace ros2_canopen
{

    const char *MasterErrorCategory::name() const noexcept
    {
        return "LelyBridgeError";
    }

    std::string MasterErrorCategory::message(int ev) const
    {
        switch (static_cast<MasterErrorCode>(ev))
        {
            case MasterErrorCode::MasterNotMasterSet:
            return "Master does not have a master object set.";
            case MasterErrorCode::MasterNotInitialised:
            return "Master is not yet intialised.";
            case MasterErrorCode::MasterNotConfigured:
            return "Master is not yet configured.";
            case MasterErrorCode::MasterNotActivated:
            return "Master is not yet activated.";
            case MasterErrorCode::MasterAlreadyMasterSet:
            return "Master does already have a master object set.";
            case MasterErrorCode::MasterAlreadyInitialised:
            return "Master is already initialised.";
            case MasterErrorCode::MasterAlreadyConfigured:
            return "Master is already configured.";
            case MasterErrorCode::MasterAlreadyActivated:
            return "Master is already activated.";
            default : 
            return "(unrecognized error)";
        }
    }
}