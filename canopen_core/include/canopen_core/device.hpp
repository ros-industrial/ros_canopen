//    Copyright 2022 Harshavadan Deshpande
//                   Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#ifndef DEVICE_HPP_
#define DEVICE_HPP_

#include <atomic>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "canopen_interfaces/srv/co_node.hpp"
#include "canopen_core/configuration_manager.hpp"

using namespace lely;

namespace ros2_canopen
{

    class DriverInterface : public rclcpp::Node
    {
    public:
        DriverInterface(const std::string &node_name,
                        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, node_options) {}

        virtual void init(ev::Executor &exec,
                          canopen::AsyncMaster &master,
                          uint8_t node_id,
                          std::shared_ptr<ros2_canopen::ConfigurationManager> config) noexcept = 0;

        virtual void remove(ev::Executor &exec,
                            canopen::AsyncMaster &master,
                            uint8_t node_id) noexcept = 0;
    };

    // Base class for driver plugin
    // Pluginlib API does allows only default constructors
    class LifecycleDriverInterface : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        /**
         * @brief Construct a new LifecycleDriverInterface object
         *
         * @param [in] node_name
         * @param [in] node_options
         */
        LifecycleDriverInterface(const std::string &node_name,
                                 const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
            : rclcpp_lifecycle::LifecycleNode(node_name, node_options) {}

        /**
         * @brief Initializer for the driver
         *
         * Initializes the driver, adds it to the CANopen Master.
         * This function needs to be executed inside the masters
         * event loop or the masters thread!
         *
         * @param [in] exec       The executor to be used for the driver
         * @param [in] master     The master the driver should be added to
         * @param [in] node_id    The nodeid of the device the driver commands
         */
        virtual void init_from_master(std::shared_ptr<ev::Executor> exec,
                                      std::shared_ptr<canopen::AsyncMaster> master,
                                      std::shared_ptr<ros2_canopen::ConfigurationManager> config)
        {
            RCLCPP_DEBUG(this->get_logger(), "init_from_master_start");
            this->exec_ = exec;
            this->master_ = master;
            this->config_ = config;
            this->initialised_ = true;
            RCLCPP_DEBUG(this->get_logger(), "init_from_master_end");
        }

        virtual void init()
        {
            RCLCPP_DEBUG(this->get_logger(), "init_start");
            this->activated_.store(false);
            register_ros_interface();
            RCLCPP_DEBUG(this->get_logger(), "init_end");
        }

        virtual bool add() = 0;
        virtual bool remove() = 0;

        /**
         * @brief Start Threads
         *
         * Used during activate transition to start threads that run tasks in
         * active state.
         *
         */
        virtual void start_threads()
        {
        }
        /**
         * @brief Join Threads
         *
         * Used during deactivate transition to join threads that run tasks in
         * active state.
         *
         */
        virtual void join_threads()
        {
        }

        virtual void update_parameters()
        {
            RCLCPP_DEBUG(this->get_logger(), "update_parameters_start");
            int millis;
            this->get_parameter("container_name", container_name_);
            this->get_parameter("non_transmit_timeout", millis);
            this->get_parameter("node_id", this->node_id_);
            this->non_transmit_timeout_ = std::chrono::milliseconds(millis);
            RCLCPP_DEBUG(this->get_logger(), "update_parameters_start");
        }

        virtual void read_config()
        {
        }

        /**
         * @brief Registers ROS Interface
         *
         * Registers the ros interface (services, topics)
         *
         */
        virtual void register_ros_interface()
        {
            RCLCPP_DEBUG(this->get_logger(), "register_ros_interface_start");
            client_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            this->declare_parameter("container_name", "");
            this->declare_parameter("node_id", 0);
            this->declare_parameter("non_transmit_timeout", 100);
            RCLCPP_DEBUG(this->get_logger(), "register_ros_interface_end");
        }

        /**
         * @brief Registers ROS Timers
         *
         * Registers ROS timers that do things during active state.
         *
         */
        virtual void
        start_timers()
        {
        }

        /**
         * @brief Registers ROS Timers
         *
         * Registers ROS timers that do things during active state.
         *
         */
        virtual void stop_timers()
        {
        }

        bool demand_init_from_master()
        {
            RCLCPP_DEBUG(this->get_logger(), "demand_init_from_master_start");
            std::string init_service_name = container_name_ + "/init_driver";
            auto demand_init_from_master_client =
                this->create_client<canopen_interfaces::srv::CONode>(
                    init_service_name,
                    rmw_qos_profile_services_default,
                    client_cbg_);

            while (!demand_init_from_master_client->wait_for_service(non_transmit_timeout_))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for init_driver service. Exiting.");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "init_driver service not available, waiting again...");
            }
            auto request = std::make_shared<canopen_interfaces::srv::CONode::Request>();
            request->nodeid = node_id_;

            auto future_result = demand_init_from_master_client->async_send_request(request);

            auto future_status = future_result.wait_for(non_transmit_timeout_);
            RCLCPP_DEBUG(this->get_logger(), "demand_init_from_master_end");
            if (future_status == std::future_status::ready)
            {
                try
                {
                    return future_result.get()->success;
                }
                catch (...)
                {
                    return false;
                }
            }
            return false;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state)
        {
            this->activated_.store(false);
            this->update_parameters();
            if (!this->demand_init_from_master())
            {
                RCLCPP_ERROR(this->get_logger(), "Demand init from master service call failed.");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            if (!this->initialised_)
            {
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            read_config();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state)
        {
            this->activated_.store(true);
            if (!this->add())
            {
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            this->start_threads();
            this->start_timers();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state)
        {
            this->activated_.store(false);
            this->stop_timers();
            this->join_threads();
            if (!this->remove())
            {
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state)
        {
            this->activated_.store(false);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state)
        {
            this->activated_.store(false);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

    protected:
        // Storing ros2 canopen objects
        std::shared_ptr<ev::Executor> exec_;
        std::shared_ptr<canopen::AsyncMaster> master_;
        std::shared_ptr<ros2_canopen::ConfigurationManager> config_;
        rclcpp::Client<canopen_interfaces::srv::CONode>::SharedPtr demand_init_from_master_client_;

        // Storing parameter values
        uint8_t node_id_;
        std::chrono::milliseconds non_transmit_timeout_;
        std::string container_name_;

        // Callback groups
        rclcpp::CallbackGroup::SharedPtr client_cbg_;
        rclcpp::CallbackGroup::SharedPtr timer_cbg_;

        // Synchronisation
        std::atomic<bool> activated_;
        bool initialised_;
    };

    class MasterInterface : public rclcpp::Node
    {
    protected:
        std::string dcf_txt_;
        std::string dcf_bin_;
        std::string can_interface_name_;
        uint8_t node_id_;
        std::shared_ptr<ros2_canopen::ConfigurationManager> config_;

    public:
        MasterInterface(
            const std::string &node_name,
            const rclcpp::NodeOptions &node_options) : rclcpp::Node(node_name, node_options)
        {
        }
        virtual void init(
            std::string dcf_txt,
            std::string dcf_bin,
            std::string can_interface_name,
            uint8_t nodeid,
            std::shared_ptr<ros2_canopen::ConfigurationManager> config)
        {
            dcf_txt_ = dcf_txt;
            dcf_bin_ = dcf_bin;
            can_interface_name_ = can_interface_name;
            node_id_ = nodeid;
            config_ = config;
        }
        virtual void add_driver(std::shared_ptr<ros2_canopen::DriverInterface>, uint8_t node_id) = 0;
        virtual void remove_driver(std::shared_ptr<ros2_canopen::DriverInterface>, uint8_t node_id) = 0;
    };

    class LifecycleMasterInterface : public rclcpp_lifecycle::LifecycleNode
    {
    protected:
        std::string dcf_txt_;
        std::string dcf_bin_;
        std::string can_interface_name_;
        uint8_t node_id_;
        std::shared_ptr<ros2_canopen::ConfigurationManager> config_;

    public:
        /**
         * @brief Construct a new Master Interface object
         *
         * @param node_name
         * @param node_options
         * @param dcf_txt
         * @param dcf_bin
         * @param can_interface_name
         * @param nodeid
         */
        LifecycleMasterInterface(
            const std::string &node_name,
            const rclcpp::NodeOptions &node_options) : rclcpp_lifecycle::LifecycleNode(node_name, node_options)
        {
        }
        virtual void init()
        {
        }

        /**
         * @brief Initialises a driver
         *
         * @param node_id
         */
        virtual void init_driver(std::shared_ptr<ros2_canopen::LifecycleDriverInterface>, uint8_t node_id) = 0;
    };

} // end ros2_canopen namespace

#endif // DEVICE_HPP_