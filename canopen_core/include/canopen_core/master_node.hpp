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
        virtual void shutdown() = 0;
        virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() = 0;
        virtual std::shared_ptr<lely::ev::Executor> get_executor() = 0;
        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;

        virtual bool is_lifecycle() = 0;
    };

    class CanopenMaster : public CanopenMasterInterface, public rclcpp::Node 
    {
    public:
        std::shared_ptr<node_interfaces::NodeCanopenMasterInterface> node_canopen_master_;
        explicit CanopenMaster(
            const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
            : rclcpp::Node("canopen_master", node_options)
        {
            //node_canopen_master_ = std::make_shared<node_interfaces::NodeCanopenMaster<rclcpp::Node>>(this);
        }

        virtual void init() override;
        virtual void shutdown() override;

        virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() override;

        virtual std::shared_ptr<lely::ev::Executor> get_executor() override;

        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
        {
            return rclcpp::Node::get_node_base_interface();
        }

        virtual bool is_lifecycle()
        {
            return false;
        }
    };

    class LifecycleCanopenMaster : public CanopenMasterInterface, public rclcpp_lifecycle::LifecycleNode
    {
    protected:
        std::shared_ptr<node_interfaces::NodeCanopenMasterInterface> node_canopen_master_;

    public:
        LifecycleCanopenMaster(
            const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
            : rclcpp_lifecycle::LifecycleNode("lifecycle_canopen_master", node_options)
        {
            node_canopen_master_ = std::make_shared<node_interfaces::NodeCanopenMaster<rclcpp_lifecycle::LifecycleNode>>(this);
        }

        virtual void init() override;
        virtual void shutdown() override;

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

        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
        {
            return rclcpp_lifecycle::LifecycleNode::get_node_base_interface();
        }
        
        virtual bool is_lifecycle()
        {
            return true;
        }
    };

}

#endif