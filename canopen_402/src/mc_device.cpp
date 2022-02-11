#ifndef MC_DEVICE_HPP
#define MC_DEVICE_HPP
#include "ros2_canopen_core/canopen_device_base.hpp"
#include "canopen_402/mc_device_node.hpp"

using namespace ros2_canopen;
namespace canopen_402
{
    /**
     * @brief Interface of Proxy Device
     * 
     * This class is the interface used by CANopenNode to 
     * load the ProxyDevice. 
     */
    class MCDevice : public CANopenDevice
    {
    public:

        void registerDriver(
            std::shared_ptr<ev::Executor> exec,
            std::shared_ptr<canopen::AsyncMaster> master,
            uint8_t id) override
        {
            /// Setup driver
            std::string node_name = "mc_device_";
            node_name.append(std::to_string(id));
            driver_ = std::make_shared<MCDeviceDriver>(*exec, *master, id);
            motor_ =  std::make_shared<Motor402>(std::string("motor"), driver_);
            driver_node_ = std::make_shared<MCDeviceNode>(node_name, driver_, motor_);
        }

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node() override
        {
            return driver_node_->get_node_base_interface();
        }

    private:
        std::shared_ptr<MCDeviceDriver> driver_;
        std::shared_ptr<MCDeviceNode> driver_node_;
        std::shared_ptr<Motor402> motor_;
    };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(canopen_402::MCDevice, ros2_canopen::CANopenDevice)

#endif