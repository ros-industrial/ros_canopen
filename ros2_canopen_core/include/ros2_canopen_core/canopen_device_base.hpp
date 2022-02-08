#ifndef CANOPEN_DEVICE_DRIVER_BASE_HPP
#define CANOPEN_DEVICE_DRIVER_BASE_HPP


#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <rclcpp/rclcpp.hpp>
using namespace lely;
namespace ros2_canopen
{
    /**
     * @brief Abstract Interface for CANopen Device
     * 
     * @details 
     * This class provides the abstract interface for implementing
     * CANopen devices on top of the #CANopenNode. It should be used
     * as base class for creating new drivers via pluginlib.
     */
    class CANopenDevice
    {
        public:
        /**
         * @brief Register a driver
         * 
         * Registers a new driver by creating the lelycore driver and the
         * corresponding ros2 node.
         * 
         * @param [in] exec Executor to be used as basis
         * @param [in] master CANopen master to be register with
         * @param [in] master_mutex CANopen master mutex used by #CANopenNode
         * @param [in] id CANopen nodeid to use for the driver
         */
        virtual void registerDriver(
            std::shared_ptr<ev::Executor> exec, 
            std::shared_ptr<canopen::AsyncMaster> master, 
            std::shared_ptr<std::mutex> master_mutex, 
            uint8_t id) = 0;

        /**
         * @brief Get the drivers ros2 nodes interface
         * 
         * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr 
         */
        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node() = 0;

        protected:
        CANopenDevice(){}
    };
}
#endif