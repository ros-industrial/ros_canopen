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
        virtual void registerDriver(
            std::shared_ptr<ev::Executor> exec, 
            std::shared_ptr<canopen::AsyncMaster> master, 
            std::shared_ptr<std::mutex> master_mutex, 
            uint8_t id) = 0;
        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node() = 0;

        protected:
        CANopenDevice(){}
    };
}
#endif