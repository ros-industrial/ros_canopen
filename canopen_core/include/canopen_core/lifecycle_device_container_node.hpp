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
#include <rclcpp/executors.hpp>
#include <rclcpp_components/component_manager.hpp>
#include "canopen_core/lifecycle_master_node.hpp"
#include "canopen_core/configuration_manager.hpp"
#include "canopen_core/lifecycle_device_manager_node.hpp"
#include "canopen_interfaces/srv/co_node.hpp"

#include "device.hpp"
/**
 * @brief Lifecyle Device Container for CANopen
 * 
 * This class provides the functionality for loading a
 * the CANopen Master and CANopen Device Drivers based on the 
 * Configuration Files. Configuration files need to be passed in
 * as paramters.
 * 
 */
class LifecycleDeviceContainerNode : public rclcpp_components::ComponentManager
{
public:

    /**
     * @brief Construct a new Lifecycle Device Container Node object
     * 
     * @param [in] executor     The executor to add loaded master and devices to.
     * @param [in] node_name    The name of the node    
     * @param [in] node_options Passed to the device_container node
     */
    LifecycleDeviceContainerNode(
        std::weak_ptr<rclcpp::Executor> executor =
            std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
        std::string node_name = "lifecycle_device_container_node",
        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()) : rclcpp_components::ComponentManager(executor, node_name, node_options)
    {

        executor_ = executor;
        this->declare_parameter<std::string>("can_interface_name", "");
        this->declare_parameter<std::string>("master_config", "");
        this->declare_parameter<std::string>("bus_config", "");
        this->declare_parameter<std::string>("master_bin", "");

        init_driver_service_ = this->create_service<canopen_interfaces::srv::CONode>(
            "~/init_driver",
            std::bind(
                &LifecycleDeviceContainerNode::on_init_driver,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        
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
    bool init();

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


private:


    
    std::map<std::string, std::pair<uint16_t, std::string>> registered_drivers_;    ///< Map of drivers registered in busconfiguration. Name is key.                        
    std::map<std::string, std::pair<uint16_t, std::string>> active_drivers_;        ///< Map of drivers activated (added to ros executor). Name is key.            
    std::map<uint16_t, rclcpp_components::NodeInstanceWrapper> node_wrappers_;      ///< Map of node instances. CAN id is key.            
    std::shared_ptr<ros2_canopen::LifecycleMasterInterface> can_master_;            ///< Pointer to can master instance
    std::shared_ptr<ev::Executor> exec_;                                            ///< Pointer to can executor instance
    std::weak_ptr<rclcpp::Executor> executor_;                                      ///< Pointer to ros executor instance
    std::shared_ptr<ros2_canopen::ConfigurationManager> config_;                    ///< Pointer to configuration manager instance
    std::string dcf_txt_;                                                           ///< Cached value of .dcf file parameter
    std::string bus_config_;                                                        ///< Cached value of bus.yml file parameter
    std::string dcf_bin_;                                                           ///< Cached value of .bin file parameter
    std::string can_interface_name_;                                                ///< Cached value of can interface name
    rclcpp::Service<canopen_interfaces::srv::CONode>::SharedPtr init_driver_service_;///< Service object for init_driver service 
    /**
     * @todo Relict??
     * 
     */          
    rclcpp::Service<canopen_interfaces::srv::CONode>::SharedPtr remove_node_master_service_;

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
     * @brief Add driver to executor
     * 
     * @param [in] driver_name  Name of the driver 
     * @param [in] node_id      CANopen node id of the target device
     * @param [in] node_name    Node name 
     */
    void add_node_to_executor(const std::string &driver_name, const uint16_t node_id, const std::string &node_name);
    void remove_node_from_executor(const std::string &driver_name, const uint16_t node_id, const std::string &node_name);

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
     * @brief Load a component
     * 
     * @param [in] package_name     Name of the package
     * @param [in] driver_name      Name of the driver class to load
     * @param [in] node_id          CANopen node id of the target device
     * @param [in] node_name        Node name of the ROS node
     * @return true 
     * @return false 
     */
    bool load_component(std::string &package_name, std::string &driver_name, uint16_t node_id, std::string &node_name);

    /**
     * @brief Returns a list of components
     *
     * @return std::map<uint16_t, std::string>
     */
    std::map<uint16_t, std::string> list_components();

    /**
     * @brief Loads drivers from configuration
     * 
     * @return true 
     * @return false 
     */
    bool load_drivers_from_config();
    /**
     * @brief Loads master from configuration
     * 
     * @return true 
     * @return false 
     */
    bool load_master_from_config();
    /**
     * @brief Loads the device manager
     * 
     * @return true 
     * @return false 
     */
    bool load_manager();

    /**
     * @brief Initialises the master
     * 
     * @param node_id 
     * @return true 
     * @return false 
     */
    bool init_master(uint16_t node_id);
};

#endif // LIFECYCLE_DEVICE_CONTAINER_NODE_HPP