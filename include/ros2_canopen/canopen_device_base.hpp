#ifndef CANOPEN_DEVICE_DRIVER_BASE_HPP
#define CANOPEN_DEVICE_DRIVER_BASE_HPP

#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>
using namespace lely;
namespace ros2_canopen
{
    class CANopenDevice
    {
        public:
        virtual void registerDriver(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id) = 0;

        protected:
        CANopenDevice(){}
    };
}
#endif