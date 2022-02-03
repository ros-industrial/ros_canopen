#ifndef BASIC_DEVICE_HPP
#define BASIC_DEVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "canopen_device_base.hpp"
#include "ros2_canopen/basic_device_driver.hpp"
#include "ros2_canopen_interfaces/msg/co_data.hpp"
#include "ros2_canopen_interfaces/srv/co_read.hpp"
#include "ros2_canopen_interfaces/srv/co_write.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_canopen
{

    class BasicDeviceNode : public rclcpp_lifecycle::LifecycleNode
    {

    private:
        std::future<void> nmt_state_publisher_future;
        std::future<void> rpdo_publisher_future;

        void nmt_listener();
        void rdpo_listener();

        CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state) override;

        CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state) override;

        CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state) override;

        CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state) override;

        CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state) override;

    protected:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver;
        std::atomic<bool> configured;


        virtual void on_nmt(canopen::NmtState nmt_state) = 0;
        virtual void on_rpdo(COData data) = 0;
        virtual CallbackReturn on_configure_app(const rclcpp_lifecycle::State &state) = 0;
        virtual CallbackReturn on_activate_app(const rclcpp_lifecycle::State &state) = 0;
        virtual CallbackReturn on_deactivate_app(const rclcpp_lifecycle::State &state) = 0;
        virtual CallbackReturn on_cleanup_app(const rclcpp_lifecycle::State &state) = 0;
        virtual CallbackReturn on_shuttdown_app(const rclcpp_lifecycle::State &state) = 0;

        explicit BasicDeviceNode(
            const std::string &node_name,
            std::shared_ptr<BasicDeviceDriver> driver,
            bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(
                  node_name,
                  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {
            this->driver = driver;
        };
    };





}

#endif