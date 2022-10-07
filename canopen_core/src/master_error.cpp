#include "canopen_core/master_error.hpp"
#include <cstring>
namespace ros2_canopen
{

    char *MasterException::what()
    {
        char * res = new char[1000];  
        strcpy(res, what_.c_str());
        return res;
    }
}