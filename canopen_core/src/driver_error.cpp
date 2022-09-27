#include "canopen_core/driver_error.hpp"

const ros2_canopen::DriverErrorCategory DriverErrorCategoryInstance{};

std::error_code std::make_error_code(ros2_canopen::DriverErrorCode e)
{
    return {static_cast<int>(e), DriverErrorCategoryInstance};
}

namespace ros2_canopen
{

    const char *DriverErrorCategory::name() const noexcept
    {
        return "LelyBridgeError";
    }

    std::string DriverErrorCategory::message(int ev) const
    {
        switch (static_cast<DriverErrorCode>(ev))
        {
            case DriverErrorCode::DriverNotMasterSet:
            return "Driver does not have a master object set.";
            case DriverErrorCode::DriverNotInitialised:
            return "Driver is not yet intialised.";
            case DriverErrorCode::DriverNotConfigured:
            return "Driver is not yet configured.";
            case DriverErrorCode::DriverNotActivated:
            return "Driver is not yet activated.";
            case DriverErrorCode::DriverAlreadyMasterSet:
            return "Driver does already have a master object set.";
            case DriverErrorCode::DriverAlreadyInitialised:
            return "Driver is already initialised.";
            case DriverErrorCode::DriverAlreadyConfigured:
            return "Driver is already configured.";
            case DriverErrorCode::DriverAlreadyActivated:
            return "Driver is already activated.";
            case DriverErrorCode::DriverFailedAddingToMaster:
            return "Driver could not be added to master.";
            case DriverErrorCode::DriverFailedRemovnigFromMaster:
            return "Driver could not be removed from master.";
            default : 
            return "(unrecognized error)";
        }
    }
}