#ifndef MASTER_NODE_HPP_
#define MASTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "canopen_core/node_interfaces/node_canopen_master.hpp"

namespace ros2_canopen
{

    class CanopenMasterInterface
    {
    public:
        virtual void init() = 0;
        virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() = 0;
        virtual std::shared_ptr<lely::ev::Executor> get_executor() = 0;
    };

    class CanopenMaster : public rclcpp::Node, public CanopenMasterInterface
    {
    public:
        std::shared_ptr<node_interfaces::NodeCanopenMasterInterface> node_canopen_master_;
        CanopenMaster(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
            : rclcpp::Node("canopen_master", node_options)
        {
            node_canopen_master_ = std::make_shared<node_interfaces::NodeCanopenMaster<rclcpp::Node>>(this);
        }

        virtual void init() override;

        virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() override;

        virtual std::shared_ptr<lely::ev::Executor> get_executor() override;

    };

    class LifecycleCanopenMaster : public rclcpp_lifecycle::LifecycleNode, public CanopenMasterInterface
    {
    protected:
        std::shared_ptr<node_interfaces::NodeCanopenMasterInterface> node_canopen_master_;

    public:
        LifecycleCanopenMaster(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
            : rclcpp_lifecycle::LifecycleNode("lifecycle_canopen_master", node_options)
        {
            node_canopen_master_ = std::make_shared<node_interfaces::NodeCanopenMaster<rclcpp_lifecycle::LifecycleNode>>(this);
        }

        virtual void init() override;

        virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() override;

        virtual std::shared_ptr<lely::ev::Executor> get_executor() override;

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