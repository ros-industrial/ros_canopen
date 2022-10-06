#ifndef DRIVER_NODE_HPP_
#define DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"

namespace ros2_canopen
{

    class CanopenDriverInterface
    {
    public:
        /**
         * @brief Initialise the driver
         * 
         * This function will initialise the drivers functionalities.
         * It will be called by the device_container when the node has
         * been added to the executor.
         */
        virtual void init() = 0;

        /**
         * @brief Set the master object
         * 
         * This function will set the Canopen Master Objects that are
         * necessary for the driver to be instantiated. It will be called
         * by the device container when the init_driver service is invoked.
         * 
         * @param exec 
         * @param master 
         */
        virtual void set_master(std::shared_ptr<lely::ev::Executor> exec,
                                std::shared_ptr<lely::canopen::AsyncMaster> master) = 0;

        /**
         * @brief Get the node base interface object
         * 
         * This function shall return an rclcpp::node_interfaces::NodeBaseInterface::SharedPtr.
         * The pointer will be used to add the driver node to the executor.
         * 
         * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr 
         */
        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;

        /**
         * @brief Shutdown the driver
         * 
         * This function will shutdown the driver and will especially
         * join all threads to enable a clean shutdown.
         * 
         */
        virtual void shutdown() = 0;

        /**
         * @brief Check whether this is a LifecycleNode
         * 
         * This function provides runtime information on whether the driver
         * is a lifecycle driver or not.
         * 
         * @return true 
         * @return false 
         */
        virtual bool is_lifecycle() = 0;

        /**
         * @brief Get the node canopen driver interface object
         * 
         * This function gives access to the underlying NodeCanopenDriverInterface.
         * 
         * @return std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> 
         */
        virtual std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> get_node_canopen_driver_interface() = 0;

    };

    class CanopenDriver : public CanopenDriverInterface, public rclcpp::Node
    {
    public:
        std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> node_canopen_driver_;
        explicit CanopenDriver(
            const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
            : rclcpp::Node("canopen_driver", node_options)
        {
            node_canopen_driver_ = std::make_shared<node_interfaces::NodeCanopenDriver<rclcpp::Node>>(this);
        }

        virtual void init() override;

        virtual void set_master(
            std::shared_ptr<lely::ev::Executor> exec,
            std::shared_ptr<lely::canopen::AsyncMaster> master) override;

        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
        {
            return rclcpp::Node::get_node_base_interface();
        }

        virtual void shutdown() override;

        virtual bool is_lifecycle() override
        {
            return false;
        }

        virtual std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> get_node_canopen_driver_interface()
        {
            return node_canopen_driver_;
        }
    };

    class LifecycleCanopenDriver : public CanopenDriverInterface, public rclcpp_lifecycle::LifecycleNode
    {
    protected:
        std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> node_canopen_driver_;

    public:
        explicit LifecycleCanopenDriver(
            const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
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

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
        {
            return rclcpp_lifecycle::LifecycleNode::get_node_base_interface();
        }
        virtual void shutdown() override;

        virtual bool is_lifecycle() override
        {
            return true;
        }

        virtual std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> get_node_canopen_driver_interface()
        {
            return node_canopen_driver_;
        }
    };

}

#endif