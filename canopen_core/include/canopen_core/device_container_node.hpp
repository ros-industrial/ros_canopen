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

#ifndef DEVICE_CONTAINER_NODE_HPP
#define DEVICE_CONTAINER_NODE_HPP

#include <memory>
#include <vector>
#include <rclcpp/executors.hpp>
#include <rclcpp_components/component_manager.hpp>
#include "canopen_core/master_node.hpp"
#include "canopen_core/configuration_manager.hpp"

#include "device.hpp"

/**
 * @brief Non Lifecyle Device Container for CANopen
 * 
 * This class provides the functionality for loading and managing
 * the CANopen Master and CANopen Device Drivers based on the 
 * Configuration Files. Configuration files need to be passed in
 * as paramters.
 * 
 */
class DeviceContainerNode : public rclcpp_components::ComponentManager {
public:

    /**
     * @brief Construct a new Device Container Node object
     * 
     * @param [in] executor     The executor to add loaded master and devices to.
     * @param [in] node_name    The name of the node    
     * @param [in] node_options Passed to the device_container node
     */
    DeviceContainerNode(
        std::weak_ptr<rclcpp::Executor> executor =
        std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
        std::string node_name = "device_manager",
        const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) :
            rclcpp_components::ComponentManager(executor, node_name, node_options) {
        
        executor_ = executor;
        this->declare_parameter<std::string>("can_interface_name", "");
        this->declare_parameter<std::string>("master_config", "");
        this->declare_parameter<std::string>("bus_config", "");
        this->declare_parameter<std::string>("master_bin", "");
    }

    /**
     * @brief Get the a CANopen Driver
     *
     * This function enables access to drivers that were loaded
     * by the device_container.  
     *
     * @param [in] node_id  Node ID of the Device the driver controls
     * @return const std::shared_ptr<void> 
     * 
     */
    const std::shared_ptr<void> get_node(size_t node_id){
        return node_wrappers_[node_id].get_node_instance();
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
     * @brief Executes the initialisation
     *
     * This will execute the intialization but read
     * Configuration parameters from function parameters
     * not ROS paramters.
     * 
     * @param can_interface_name 
     * @param master_config 
     * @param bus_config 
     * @param master_bin 
     * @return true 
     * @return false 
     */
    bool init(const std::string& can_interface_name,
              const std::string& master_config,
              const std::string& bus_config,
              const std::string& master_bin="");


    /**
     * @brief Get the node instance wrapper map object
     * 
     * @return std::map<uint8_t, rclcpp_components::NodeInstanceWrapper> 
     */
    std::map<uint8_t, rclcpp_components::NodeInstanceWrapper> get_node_instance_wrapper_map(){
        return node_wrappers_;
    }

    /**
     * @brief Get the active drivers object
     * 
     * @return std::map<std::string, std::pair<uint8_t, std::string>> 
     */
    std::map<std::string, std::pair<uint8_t, std::string>> get_active_drivers(){
        return active_drivers_;
    }

    /**
     * @brief Get the registered drivers object
     * 
     * @return std::map<std::string, std::pair<uint8_t, std::string>> 
     */
    std::map<std::string, std::pair<uint8_t, std::string>> get_registered_drivers(){
        return registered_drivers_;
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


private:
    std::map<std::string, std::pair<uint8_t, std::string>> registered_drivers_; ///< Map of drivers registered in busconfiguration. Name is key.
    std::map<std::string, std::pair<uint8_t, std::string>> active_drivers_;     ///< Map of drivers activated. Name is key.
    std::shared_ptr<ros2_canopen::MasterInterface> can_master_;                 ///< Pointer to can master instance
    std::shared_ptr<ev::Executor> exec_;                                        ///< Pointer to can executor instance
    std::weak_ptr<rclcpp::Executor> executor_;                                  ///< Pointer to ros executor instance
    std::map<uint8_t, rclcpp_components::NodeInstanceWrapper> node_wrappers_;   ///< Map of node instances that are in the executor. CAN id is key.
    std::shared_ptr<ros2_canopen::ConfigurationManager> config_;                ///< Pointer to configuration manager instance
    std::string dcf_txt_;                                                       ///< Cached value of .dcf file parameter
    std::string bus_config_;                                                    ///< Cached value of bus.yml file parameter
    std::string dcf_bin_;                                                       ///< Cached value of .bin file parameter
    std::string can_interface_name_;                                            ///< Cached value of can interface name

    /**
     * @brief Set the ROS executor object
     * 
     * @param [in] executor 
     */
    void set_executor(const std::weak_ptr<rclcpp::Executor> executor);

    /**
     * @brief Adds a driver to the ROS executor
     * 
     * @param [in] driver_name  Name of the driver 
     * @param [in] node_id      CANopen node Id of the target device
     * @param [in] node_name    Node name 
     */
    void add_node_to_executor(const std::string &driver_name, const uint8_t node_id, const std::string &node_name);

    /**
     * @brief Removes a driver from the ROS executor
     * 
     * @param [in] driver_name  Name of the driver 
     * @param [in] node_id      CANopen node Id of the target device 
     * @param [in] node_name    Node name 
     */
    void remove_node_from_executor(const std::string &driver_name, const uint8_t node_id, const std::string &node_name);

    /**
     * @brief Adds driver to master
     * 
     * This function needs to be called to add the driver to the
     * CANopen master loop, so that it has access to CANopen events
     * and can send messages. 
     * 
     * @param [in] driver_name   Name of the driver
     * @param [in] node_id       CANopen node Id of the target device
     * 
     * 
     */
    void add_driver_to_master(std::string driver_name, uint8_t node_id);

    /**
     * @brief Removes driver from master
     * 
     * This function removes the driver with the specified id from
     * the CANopen master loop. This needs to be done when unloading
     * a driver. This function should be called before removing the driver
     * from the ros2 executor.
     * 
     * @param node_id       CANopen node Id of the target device
     */
    void remove_driver_from_master(uint8_t node_id);


    /**
     * @brief Load a component
     * 
     * @param [in] package_name     Name of the package
     * @param [in] driver_name      Name of the driver class to load
     * @param [in] node_id          CANopen Node id of the target device
     * @param [in] node_name        Node name of the ROS node
     * @return true 
     * @return false 
     */
    bool load_component(std::string& package_name, std::string& driver_name, uint8_t node_id, std::string& node_name);

    /**
     * @brief Get a list of loaded components
     * 
     * @return std::map<uint32_t, std::string> 
     */
    std::map<uint32_t, std::string> list_components();

    /**
     * @brief Initialize all components that are in the configuration.
     * 
     * @return true 
     * @return false 
     */
    bool init_devices_from_config();

    /**
     * @brief Add the master to the ROS executor
     * 
     * @param [in] node_id  CANopen node id of the master
     * @return true 
     * @return false 
     */
    bool add_master(uint8_t node_id);
};

#endif // DEVICE_CONTAINER_NODE_HPP
