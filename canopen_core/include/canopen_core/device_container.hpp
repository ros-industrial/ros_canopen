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

#ifndef LIFECYCLE_DEVICE_CONTAINER_NODE_HPP
#define LIFECYCLE_DEVICE_CONTAINER_NODE_HPP

#include <memory>
#include <vector>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp_components/component_manager.hpp>
#include "canopen_core/configuration_manager.hpp"
#include "canopen_interfaces/srv/co_node.hpp"
#include "canopen_core/driver_node.hpp"
#include "canopen_core/master_node.hpp"
#include "canopen_core/lifecycle_manager.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using namespace std::chrono_literals;

namespace ros2_canopen
{
    /**
     * @brief Lifecyle Device Container for CANopen
     *
     * This class provides the functionality for loading a
     * the CANopen Master and CANopen Device Drivers based on the
     * Configuration Files. Configuration files need to be passed in
     * as paramters.
     *
     */
    class DeviceContainer : public rclcpp_components::ComponentManager
    {
    public:
        /**
         * @brief Construct a new Lifecycle Device Container Node object
         *
         * @param [in] executor     The executor to add loaded master and devices to.
         * @param [in] node_name    The name of the node
         * @param [in] node_options Passed to the device_container node
         */
        DeviceContainer(
            std::weak_ptr<rclcpp::Executor> executor =
                std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
            std::string node_name = "device_container",
            const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()) : rclcpp_components::ComponentManager(executor, node_name, node_options)
        {

            executor_ = executor;
            this->declare_parameter<std::string>("can_interface_name", "");
            this->declare_parameter<std::string>("master_config", "");
            this->declare_parameter<std::string>("bus_config", "");
            this->declare_parameter<std::string>("master_bin", "");
            client_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            init_driver_service_ = this->create_service<canopen_interfaces::srv::CONode>(
                "~/init_driver",
                std::bind(
                    &DeviceContainer::on_init_driver,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
                rmw_qos_profile_services_default,
                client_cbg_);

            this->loadNode_srv_.reset();
            this->unloadNode_srv_.reset();
            lifecycle_operation_ = false;
        }

        /**
         * @brief Executes the intialisation
         *
         * This will read nodes parameters and initialize
         * the configuration defined in the paramters. It
         * will also start loading the components.
         *
         * @return true
         * @return false
         */
        void init();

        void init(const std::string &can_interface_name,
                  const std::string &master_config,
                  const std::string &bus_config,
                  const std::string &master_bin = "");

        virtual void configure();
        /**
         * @brief Loads drivers from configuration
         *
         * @return true
         * @return false
         */
        virtual bool load_drivers();
        /**
         * @brief Loads master from configuration
         *
         * @return true
         * @return false
         */
        virtual bool load_master();
        /**
         * @brief Loads the device manager
         *
         * @return true
         * @return false
         */
        virtual bool load_manager();

        /**
         * @brief Load a component
         *
         * @param [in] package_name     Name of the package
         * @param [in] driver_name      Name of the driver class to load
         * @param [in] node_id          CANopen node id of the target device
         * @param [in] node_name        Node name of the ROS node
         * @return true
         * @return false
         */
        virtual bool load_component(
            std::string &package_name,
            std::string &driver_name,
            uint16_t node_id,
            std::string &node_name,
            std::vector<rclcpp::Parameter> &params);

        /**
         * @brief Shutdown all devices.
         *
         */
        virtual void shutdown()
        {
            for (auto it = registered_drivers_.begin(); it != registered_drivers_.end(); ++it)
            {
                try
                {
                    it->second->shutdown();
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
            }
            try
            {
                can_master_->shutdown();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
            
        }
        /**
         * @brief Callback for the listing service
         *
         * @param request_header
         * @param request
         * @param response
         */
        virtual void
        on_list_nodes(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<ListNodes::Request> request,
            std::shared_ptr<ListNodes::Response> response) override;

        /**
         * @brief Get the registered drivers object
         *
         * @return std::map<uint16_t, std::shared_ptr<CanopenDriverInterface>>
         */
        virtual std::map<uint16_t, std::shared_ptr<CanopenDriverInterface>> get_registered_drivers()
        {
            return registered_drivers_;
        }

        /**
         * @brief Get the number of registered drivers
         *
         * @return size_t
         */
        virtual size_t count_drivers()
        {
            return registered_drivers_.size();
        }

        /**
         * @brief Get the ids of drivers with type object
         *
         * @param type
         * @param ids
         */
        std::vector<uint16_t> get_ids_of_drivers_with_type(std::string type)
        {
            std::vector<std::string> devices;
            std::vector<uint16_t> ids;
            uint32_t count = this->config_->get_all_devices(devices);

            for (auto it = devices.begin(); it != devices.end(); it++)
            {
                auto driver_name = config_->get_entry<std::string>(*it, "driver");
                if (driver_name.has_value())
                {
                    std::string name = driver_name.value();
                    if (name.compare(type) == 0)
                    {
                        auto node_id = config_->get_entry<uint16_t>(*it, "node_id");
                        ids.push_back(node_id.value());
                    }
                }
            }
            return ids;
        }

        std::string get_driver_type(uint16_t id)
        {
            std::vector<std::string> devices;
            uint32_t count = this->config_->get_all_devices(devices);
            for (auto it = devices.begin(); it != devices.end(); it++)
            {
                auto node_id = config_->get_entry<uint16_t>(*it, "node_id");
                if(node_id.has_value() && node_id.value() == id)
                {
                    auto driver_name = config_->get_entry<std::string>(*it, "driver");
                    return driver_name.value();
                }
            }
        }

    protected:
        // Components
        std::map<uint16_t, std::shared_ptr<CanopenDriverInterface>> registered_drivers_; ///< Map of drivers registered in busconfiguration. Name is key.
        std::shared_ptr<ros2_canopen::CanopenMasterInterface> can_master_;               ///< Pointer to can master instance
        uint16_t can_master_id_;
        std::unique_ptr<ros2_canopen::LifecycleManager> lifecycle_manager_;

        // Configuration
        std::shared_ptr<ros2_canopen::ConfigurationManager> config_; ///< Pointer to configuration manager instance
        std::string dcf_txt_;                                        ///< Cached value of .dcf file parameter
        std::string bus_config_;                                     ///< Cached value of bus.yml file parameter
        std::string dcf_bin_;                                        ///< Cached value of .bin file parameter
        std::string can_interface_name_;                                  ///< Cached value of can interface name
        bool lifecycle_operation_;

        // ROS Objects
        std::weak_ptr<rclcpp::Executor> executor_;                                        ///< Pointer to ros executor instance
        rclcpp::Service<canopen_interfaces::srv::CONode>::SharedPtr init_driver_service_; ///< Service object for init_driver service
        rclcpp::CallbackGroup::SharedPtr client_cbg_;                                     ///< Callback group for services

        /**
         * @brief Initialize the device manager
         *
         * @param [in] node_id  CANopen node id of the device manager
         * @return true
         * @return false
         */
        bool init_device_manager(uint16_t node_id);

        /**
         * @brief Set the ROS executor object
         *
         * @param [in] executor     Pointer to the Executor
         */
        void set_executor(const std::weak_ptr<rclcpp::Executor> executor);

        /**
         * @brief Adds driver to master
         *
         * This function needs to be called to add the driver to the
         * CANopen master loop, so that it has access to CANopen events
         * and can send messages.
         *
         * @param node_id       CANopen node id of the target device
         *
         * @return true
         * @return false
         */
        bool init_driver(uint16_t node_id);

        /**
         * @brief Callback for init driver service
         *
         * @param request
         * @param response
         */
        void on_init_driver(
            const canopen_interfaces::srv::CONode::Request::SharedPtr request,
            canopen_interfaces::srv::CONode::Response::SharedPtr response)
        {
            response->success = init_driver(request->nodeid);
        }



        /**
         * @brief Returns a list of components
         *
         * @return std::map<uint16_t, std::string>
         */
        std::map<uint16_t, std::string> list_components();

        /**
         * @brief Add driver to executor
         *
         * @param [in] driver_name  Name of the driver
         * @param [in] node_id      CANopen node id of the target device
         * @param [in] node_name    Node name
         */
        virtual void add_node_to_executor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface)
        {
            if (auto exec = executor_.lock())
            {
                RCLCPP_INFO(this->get_logger(),
                            "Added %s to executor",
                            node_interface->get_fully_qualified_name());
                exec->add_node(node_interface, true);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to add component %s", node_interface->get_fully_qualified_name());
            }
        }

        // bool set_remote_paramter(std::string node_name, rclcpp::Parameter& param);
    };

}
#endif // LIFECYCLE_DEVICE_CONTAINER_NODE_HPP