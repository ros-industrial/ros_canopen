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

class LifecycleDeviceContainerNode : public rclcpp_components::ComponentManager
{
public:
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

    bool init();

    virtual void
    on_list_nodes(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ListNodes::Request> request,
        std::shared_ptr<ListNodes::Response> response) override;


private:
    // Stores registered drivers as device_name, pair(node_id, driver_name)
    std::map<std::string, std::pair<uint16_t, std::string>> registered_drivers_;
    // Stores drivers that were succesfully added to executor as device_name, pair(node_id, driver_name)
    std::map<std::string, std::pair<uint16_t, std::string>> active_drivers_;
    // Stores componentes as node_id and wrapper_object
    std::map<uint16_t, rclcpp_components::NodeInstanceWrapper> node_wrappers_;
    std::shared_ptr<ros2_canopen::LifecycleMasterInterface> can_master_;
    std::shared_ptr<ev::Executor> exec_;
    std::weak_ptr<rclcpp::Executor> executor_;
    std::shared_ptr<ros2_canopen::ConfigurationManager> config_;
    std::string dcf_txt_;
    std::string bus_config_;
    std::string dcf_bin_;
    std::string can_interface_name_;

    rclcpp::Service<canopen_interfaces::srv::CONode>::SharedPtr init_driver_service_;
    rclcpp::Service<canopen_interfaces::srv::CONode>::SharedPtr remove_node_master_service_;
    bool init_device_manager(uint16_t node_id);
    void set_executor(const std::weak_ptr<rclcpp::Executor> executor);
    void add_node_to_executor(const std::string &driver_name, const uint16_t node_id, const std::string &node_name);
    void remove_node_from_executor(const std::string &driver_name, const uint16_t node_id, const std::string &node_name);

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
    bool init_driver(uint16_t node_id);

    void on_init_driver(
        const canopen_interfaces::srv::CONode::Request::SharedPtr request,
        canopen_interfaces::srv::CONode::Response::SharedPtr response)
    {
        response->success = init_driver(request->nodeid);
    }

    bool load_component(std::string &package_name, std::string &driver_name, uint16_t node_id, std::string &node_name);

    /**
     * @brief Returns a list of components
     *
     * @return std::map<uint16_t, std::string>
     */
    std::map<uint16_t, std::string> list_components();

    bool load_drivers_from_config();
    bool load_master_from_config();
    bool load_manager();

    bool init_master(uint16_t node_id);
};

#endif // LIFECYCLE_DEVICE_CONTAINER_NODE_HPP