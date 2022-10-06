#include "canopen_core/device_container_error.hpp"
#include <cstring>
namespace ros2_canopen
{

    char *DeviceContainerException::what()
    {
        char * res = new char[1000];  
        strcpy(res, what_.c_str());
        return res;
    }
}