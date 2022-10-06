
#ifndef SLAVE_HPP
#define SLAVE_HPP
#include <atomic>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_canopen
{
    class BaseSlave : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit BaseSlave(const std::string &node_name, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(node_name,
                                              rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {
            this->declare_parameter("node_id", 2);
            this->declare_parameter("slave_config", "slave.eds");
            this->declare_parameter("can_interface_name", "vcan0");
            this->activated.store(false);
        }

        virtual void run() = 0;

    protected:
        std::thread run_thread;
        int node_id_;
        std::string slave_config_;
        std::string can_interface_name_;
        std::atomic<bool> activated;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            this->activated.store(false);
            RCLCPP_INFO(this->get_logger(), "Reaching inactive state.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            this->activated.store(true);
            get_parameter("node_id", node_id_);
            get_parameter("slave_config", slave_config_);
            get_parameter("can_interface_name", can_interface_name_);
            run_thread = std::thread(std::bind(&BaseSlave::run, this));
            RCLCPP_INFO(this->get_logger(), "Reaching active state.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            this->activated.store(false);
            RCLCPP_INFO(this->get_logger(), "Reaching inactive state.");
            if(run_thread.joinable())
            {
                run_thread.join();
            }
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            this->activated.store(false);
            RCLCPP_INFO(this->get_logger(), "Reaching unconfigured state.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
    };
}

#endif