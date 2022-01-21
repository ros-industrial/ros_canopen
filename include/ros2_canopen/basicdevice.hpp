#ifndef BASIC_DEVICE_HPP
#define BASIC_DEVICE_HPP
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "canopen_device_base.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_canopen
{
    class BasicDriverNode : public rclcpp_lifecycle::LifecycleNode
    {
    private:



    public:
        explicit BasicDriverNode(const std::string &node_name, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(
                  node_name,
                  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {

        }

        CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state)
        {

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state)
        {
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state)
        {
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state)
        {
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state)
        {
            return CallbackReturn::SUCCESS;
        }
    };

    class BasicDeviceDriver : canopen::FiberDriver
    {
    public:
        using FiberDriver::FiberDriver;
        BasicDeviceDriver(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id) : FiberDriver(exec, master, id)
        {

        }

    private:
        void
        OnBoot(canopen::NmtState state, char es,
               const std::string &whatisit) noexcept override
               {}


        void
        OnConfig(std::function<void(std::error_code ec)> res) noexcept override
        {}

        void
        OnState(canopen::NmtState state) noexcept override
        {}

        void
        OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
        {}

        void
        OnEmcy(uint16_t eec, uint8_t er, uint8_t *msef) noexcept override
        {}
    };

    class BasicDevice : public CANopenDevice
    {
    public:
        void registerDriver(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id)
        {
            /// Setup driver
            driver_ = std::make_unique<BasicDeviceDriver>(exec, master, id);
        }

    private:
        std::unique_ptr<BasicDeviceDriver> driver_;
    };
}

#endif