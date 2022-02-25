#ifndef BASE_DEVICE_DRIVER__BASE_DEVICE_DRIVER_HPP_
#define BASE_DEVICE_DRIVER__BASE_DEVICE_DRIVER_HPP_

#include "base_device_driver/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "base_device_driver/lely_bridge.hpp"
#include "ros2_canopen_core/device.hpp"
#include "ros2_canopen_interfaces/msg/co_data.hpp"
#include "ros2_canopen_interfaces/srv/co_read.hpp"
#include "ros2_canopen_interfaces/srv/co_write.hpp"


namespace ros2_canopen
{
    /**
     * @brief Abstract Class for a DeviceNode
     * 
     * This class provides the base functionality for creating a
     * CANopen device node. It provides callbacks for nmt and rpdo.
     */
    class BaseDeviceDriver : public CANopenDriverWrapper
    {

    private:
        std::future<void> nmt_state_publisher_future;
        std::future<void> rpdo_publisher_future;

        void nmt_listener();
        void rdpo_listener();

    protected:
        std::shared_ptr<ros2_canopen::LelyBridge> driver;

        /**
         * @brief NMT State Change Callback
         * 
         * This function is called, when the NMT State of the
         * associated LelyBridge changes,
         * 
         * @param [in] nmt_state New NMT state
         */
        virtual void on_nmt(canopen::NmtState nmt_state)
        {
            RCLCPP_INFO(this->get_logger(), "on_nmt not implemented");
        }

        /**
         * @brief RPDO Callback
         * 
         * This funciton is called when the associated 
         * LelyBridge detects a change 
         * on a specific object, due to an RPDO event.
         * 
         * @param [in] data Changed object
         */
        virtual void on_rpdo(COData data)
        {
            RCLCPP_INFO(this->get_logger(), "on_rpdo not implemented");
        }

 
        explicit BaseDeviceDriver(
            const rclcpp::NodeOptions & options) : CANopenDriverWrapper("base_driver",options) {}
    
    public:
        void init(ev::Executor& exec,
            canopen::AsyncMaster& master,
            uint8_t node_id) noexcept override;
    };
}

#endif
