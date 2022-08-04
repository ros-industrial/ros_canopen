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

class DeviceContainerNode : public rclcpp_components::ComponentManager {
public:
    DeviceContainerNode(
        std::weak_ptr<rclcpp::Executor> executor =
        std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
        std::string node_name = "device_manager",
        const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()
            .start_parameter_services(false)
            .start_parameter_event_publisher(false)) :
            rclcpp_components::ComponentManager(executor, node_name, node_options) {
        
        executor_ = executor;
        this->declare_parameter<std::string>("can_interface_name", "");
        this->declare_parameter<std::string>("master_config", "");
        this->declare_parameter<std::string>("bus_config", "");
        this->declare_parameter<std::string>("master_bin", "");
    }

    const std::shared_ptr<void> get_node(size_t node_id){
        return node_wrappers_[node_id].get_node_instance();

    }

    bool init();

    bool init(const std::string& can_interface_name,
              const std::string& master_config,
              const std::string& bus_config,
              const std::string& master_bin="");

    std::map<uint8_t, rclcpp_components::NodeInstanceWrapper> get_node_instance_wrapper_map(){
        return node_wrappers_;
    }

    std::map<std::string, std::pair<uint8_t, std::string>> get_active_drivers(){
        return active_drivers_;
    }
    std::map<std::string, std::pair<uint8_t, std::string>> get_registered_drivers(){
        return registered_drivers_;
    }

    virtual void
    on_load_node(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<LoadNode::Request> request,
        std::shared_ptr<LoadNode::Response> response);

    virtual void
    OnLoadNode(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<LoadNode::Request> request,
        std::shared_ptr<LoadNode::Response> response) override
    {
        on_load_node(request_header, request, response);
    }

    virtual void
    on_unload_node(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<UnloadNode::Request> request,
        std::shared_ptr<UnloadNode::Response> response);

    virtual void
    OnUnloadNode(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<UnloadNode::Request> request,
        std::shared_ptr<UnloadNode::Response> response) override
    {
        on_unload_node(request_header, request, response);
    }

    virtual void
    on_list_nodes(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ListNodes::Request> request,
        std::shared_ptr<ListNodes::Response> response);

    virtual void
    OnListNodes(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ListNodes::Request> request,
        std::shared_ptr<ListNodes::Response> response) override
    {
        on_list_nodes(request_header, request, response);
    }


private:
    std::map<std::string, std::pair<uint8_t, std::string>> registered_drivers_;
    std::map<std::string, std::pair<uint8_t, std::string>> active_drivers_;
    std::shared_ptr<ros2_canopen::MasterInterface> can_master_;
    std::shared_ptr<ev::Executor> exec_;
    std::weak_ptr<rclcpp::Executor> executor_;
    std::map<uint8_t, rclcpp_components::NodeInstanceWrapper> node_wrappers_;
    std::shared_ptr<ros2_canopen::ConfigurationManager> config_;
    std::string dcf_txt_;
    std::string bus_config_;
    std::string dcf_bin_;
    std::string can_interface_name_;

    void set_executor(const std::weak_ptr<rclcpp::Executor> executor);
    void add_node_to_executor(const std::string &driver_name, const uint8_t node_id, const std::string &node_name);
    void remove_node_from_executor(const std::string &driver_name, const uint8_t node_id, const std::string &node_name);

    /**
     * @brief Adds driver to master
     * 
     * This function needs to be called to add the driver to the
     * CANopen master loop, so that it has access to CANopen events
     * and can send messages. 
     * 
     * @param driver_name   Name of the driver
     * @param node_id       CANopen Id of the device the driver targets
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
     * @param node_id       CANopen Id of the device the driver targets
     */
    void remove_driver_from_master(uint8_t node_id);

    bool load_component(std::string& package_name, std::string& driver_name, uint8_t node_id, std::string& node_name);
    std::map<uint32_t, std::string> list_components();

    bool init_devices_from_config();
    bool add_master(uint8_t node_id);
};

#endif // DEVICE_CONTAINER_NODE_HPP
