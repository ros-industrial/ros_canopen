#ifndef BASIC_DEVICE_HPP
#define BASIC_DEVICE_HPP
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "canopen_device_base.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_canopen
{
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

    class BasicDriverNode : public rclcpp_lifecycle::LifecycleNode
    {
    private:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver;
        std::shared_ptr<std::mutex> master_mutex;


    public:
        explicit BasicDriverNode(
            const std::string &node_name,                   
            std::shared_ptr<BasicDeviceDriver> driver,
            std::shared_ptr<std::mutex> master_mutex,
            bool intra_process_comms = false
            )
            : rclcpp_lifecycle::LifecycleNode(
                  node_name,
                  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {
            this->driver = driver;
            this->master_mutex = master_mutex;
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

    class BasicDevice : public CANopenDevice
    {
    public:
        void registerDriver(ev_exec_t *exec, canopen::AsyncMaster &master, std::shared_ptr<std::mutex> master_mutex, uint8_t id)
        {
            /// Setup driver
            std::string node_name = "BasicDevice";
            node_name.append(std::to_string(id));
            driver_ = std::make_shared<BasicDeviceDriver>(exec, master, id);
            driver_node_ = std::make_shared<BasicDriverNode>(node_name, driver_, master_mutex);
        }

        std::shared_ptr<BasicDriverNode> get_node(){
            return driver_node_;
        }

    private:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver_;
        std::shared_ptr<ros2_canopen::BasicDriverNode> driver_node_;
    };
}

#endif