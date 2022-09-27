#ifndef DRIVER_NODE_HPP_
#define DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"

namespace ros2_canopen
{

    class CanopenDriverInterface
    {
    public:
        virtual void init() = 0;
        virtual void set_master(std::shared_ptr<lely::ev::Executor> exec,
                        std::shared_ptr<lely::canopen::AsyncMaster> master) = 0;
    };

    class CanopenDriver : public rclcpp::Node, public CanopenDriverInterface
    {
    public:
        std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> node_canopen_driver_;
        CanopenDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
            : rclcpp::Node("canopen_driver", node_options)
        {
            node_canopen_driver_ = std::make_shared<node_interfaces::NodeCanopenDriver<rclcpp::Node>>(this);
        }

        virtual void init() override;

        virtual void set_master(
            std::shared_ptr<lely::ev::Executor> exec,
            std::shared_ptr<lely::canopen::AsyncMaster> master) override;
    };

    class LifecycleCanopenDriver : public rclcpp_lifecycle::LifecycleNode, public CanopenDriverInterface
    {
    protected:
        std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> node_canopen_driver_;
    public:
        LifecycleCanopenDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
            : rclcpp_lifecycle::LifecycleNode("lifecycle_canopen_driver", node_options)
        {
            node_canopen_driver_ = std::make_shared<node_interfaces::NodeCanopenDriver<rclcpp_lifecycle::LifecycleNode>>(this);
        }

        virtual void init() override;

        virtual void set_master(
            std::shared_ptr<lely::ev::Executor> exec,
            std::shared_ptr<lely::canopen::AsyncMaster> master) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state);
    };

}

#endif