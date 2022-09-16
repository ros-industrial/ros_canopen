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

#include "canopen_core/lifecycle_device_container_node.hpp"

using namespace ros2_canopen;

void LifecycleDeviceContainerNode::set_executor(const std::weak_ptr<rclcpp::Executor> executor)
{
    executor_ = executor;
}

void LifecycleDeviceContainerNode::add_node_to_executor(const std::string &driver_name, const uint16_t node_id, const std::string &node_name)
{
    if (auto exec = executor_.lock())
    {
        exec->add_node(node_wrappers_[node_id].get_node_base_interface(), true);

        auto node_instance = std::static_pointer_cast<ros2_canopen::LifecycleDriverInterface>(node_wrappers_[node_id].get_node_instance());

        active_drivers_.insert({node_name, std::make_pair(node_id, driver_name)});

        RCLCPP_INFO(this->get_logger(), "Added node of type %s with name \"%s\" for node_id %hu to executor.", driver_name.c_str(), node_name.c_str(), node_id);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to add %s of type %s to executor", node_name.c_str(), driver_name.c_str());
    }
}

void LifecycleDeviceContainerNode::remove_node_from_executor(const std::string &driver_name, const uint16_t node_id, const std::string &node_name)
{
    RCLCPP_INFO(this->get_logger(), "Removing %s", driver_name.c_str());
    if (auto exec = executor_.lock())
    {
        exec->remove_node(node_wrappers_[node_id].get_node_base_interface());

        auto node_instance = std::static_pointer_cast<ros2_canopen::LifecycleDriverInterface>(node_wrappers_[node_id].get_node_instance());

        RCLCPP_INFO(this->get_logger(), "Removed %s of type %s from executor", node_name.c_str(), driver_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to remove %s of type %s from executor", node_name.c_str(), driver_name.c_str());
    }
}

bool LifecycleDeviceContainerNode::init_driver(uint16_t node_id)
{
    auto node_instance = std::static_pointer_cast<ros2_canopen::LifecycleDriverInterface>(node_wrappers_[node_id].get_node_instance());
    can_master_->init_driver(node_instance, node_id);
    return true;
}

bool LifecycleDeviceContainerNode::load_component(std::string &package_name, std::string &driver_name, uint16_t node_id, std::string &node_name)
{
    ComponentResource component;
    std::vector<ComponentResource> components = this->get_component_resources(package_name);
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        if (it->first.compare(driver_name) == 0)
        {
            auto factory_node = this->create_component_factory(*it);
            rclcpp::NodeOptions opts;
            opts.use_global_arguments(false);
            std::vector<std::string> remap_rules;
            remap_rules.push_back("--ros-args");
            remap_rules.push_back("-r");
            remap_rules.push_back("__node:=" + node_name);
            opts.arguments(remap_rules);

            try
            {
                node_wrappers_[node_id] = factory_node->create_node_instance(opts);
            }
            catch (const std::exception &ex)
            {
                // In the case that the component constructor throws an exception,
                // rethrow into the following catch block.
                throw rclcpp_components::ComponentManagerException(
                    "Component constructor threw an exception: " + std::string(ex.what()));
            }
            catch (...)
            {
                // In the case that the component constructor throws an exception,
                // rethrow into the following catch block.
                throw rclcpp_components::ComponentManagerException("Component constructor threw an exception");
            }

            return true;
        }
    }
    return false;
}

std::map<uint16_t, std::string> LifecycleDeviceContainerNode::list_components()
{
    std::map<uint16_t, std::string> components;
    for (auto &wrapper : node_wrappers_)
    {
        components[wrapper.first] =
            wrapper.second.get_node_base_interface()->get_fully_qualified_name();
    }
    return components;
}

bool LifecycleDeviceContainerNode::init_master(uint16_t node_id)
{
    RCLCPP_INFO(this->get_logger(), "Adding master with node id %u", node_id);
    can_master_ = std::static_pointer_cast<ros2_canopen::LifecycleMasterInterface>(node_wrappers_[node_id].get_node_instance());
    can_master_->init();
    can_master_->set_parameter(rclcpp::Parameter("master_config", this->dcf_txt_));
    can_master_->set_parameter(rclcpp::Parameter("master_bin", this->dcf_bin_));
    can_master_->set_parameter(rclcpp::Parameter("can_interface_name", this->can_interface_name_));
    can_master_->set_parameter(rclcpp::Parameter("node_id", node_id));
    RCLCPP_INFO(this->get_logger(), "Added master with node id %u", node_id);
    return true;
}

bool LifecycleDeviceContainerNode::init_device_manager(uint16_t node_id)
{
    RCLCPP_INFO(this->get_logger(), "Initialising device_manager with node id %u", node_id);
    auto device_manager = std::static_pointer_cast<ros2_canopen::LifecycleDeviceManagerNode>(node_wrappers_[node_id].get_node_instance());
    device_manager->init(this->config_);
    device_manager->set_parameter(rclcpp::Parameter("container_name", this->get_fully_qualified_name()));
    return true;
}

bool LifecycleDeviceContainerNode::load_master_from_config()
{
    RCLCPP_INFO(this->get_logger(), "Loading Master Configuration.");
    std::vector<std::string> devices;
    uint32_t count = this->config_->get_all_devices(devices);
    bool master_found = false;

    // Find master in configuration
    for (auto it = devices.begin(); it != devices.end(); it++)
    {
        if (it->find("master") != std::string::npos && !master_found)
        {
            auto node_id = config_->get_entry<uint16_t>(*it, "node_id");
            auto driver_name = config_->get_entry<std::string>(*it, "driver");
            auto package_name = config_->get_entry<std::string>(*it, "package");

            if (!node_id.has_value() || !driver_name.has_value() || !package_name.has_value())
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has uncomplete configuration for master");
                return false;
            }
            auto res = this->registered_drivers_.emplace(*it, std::make_pair(node_id.value(), driver_name.value()));
            if (!res.second)
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has duplicate configuration for %s", it->c_str());
                return false;
            }

            if (!this->load_component(package_name.value(), driver_name.value(), node_id.value(), *it))
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Loading master failed.");
                return false;
            }
            add_node_to_executor(driver_name.value(), node_id.value(), *it);
            init_master(node_id.value());
            master_found = true;
        }
    }

    if (!master_found)
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Master not in configuration");
        return false;
    }
    return true;
}

bool LifecycleDeviceContainerNode::load_drivers_from_config()
{
    RCLCPP_INFO(this->get_logger(), "Loading Driver Configuration.");
    std::vector<std::string> devices;
    uint32_t count = this->config_->get_all_devices(devices);
    for (auto it = devices.begin(); it != devices.end(); it++)
    {
        if (it->find("master") == std::string::npos)
        {
            auto node_id = config_->get_entry<uint16_t>(*it, "node_id");
            auto driver_name = config_->get_entry<std::string>(*it, "driver");
            auto package_name = config_->get_entry<std::string>(*it, "package");
            if (!node_id.has_value() || !driver_name.has_value() || !package_name.has_value())
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has uncomplete configuration for %s", it->c_str());
                return false;
            }
            auto res = this->registered_drivers_.emplace(*it, std::make_pair(node_id.value(), driver_name.value()));
            if (!res.second)
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has duplicate configuration for %s", it->c_str());
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "Found device %s with driver %s", it->c_str(), driver_name.value().c_str());

            if (!this->load_component(package_name.value(), driver_name.value(), node_id.value(), *it))
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Loading master failed.");
                return false;
            }

            add_node_to_executor(driver_name.value(), node_id.value(), *it);
            auto node_instance = std::static_pointer_cast<ros2_canopen::LifecycleDriverInterface>(node_wrappers_[node_id.value()].get_node_instance());
            node_instance->init();
            node_instance->set_parameter(rclcpp::Parameter("node_id", node_id.value()));
            node_instance->set_parameter(rclcpp::Parameter("container_name", this->get_fully_qualified_name()));
            //init_driver(node_id.value());
        }
    }
    return true;
}

bool LifecycleDeviceContainerNode::load_manager()
{
    RCLCPP_INFO(this->get_logger(), "Loading Manager Configuration.");
    std::string package = "canopen_core";
    std::string driver = "ros2_canopen::LifecycleDeviceManagerNode";
    uint16_t id = 256;
    std::string name = "lifecycle_device_manager_node";
    if (!this->load_component(package, driver, 256, name))
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Loading device_manager failed.");
        return false;
    }
    add_node_to_executor(driver, id, name);
    init_device_manager(id);
    return true;
}

bool LifecycleDeviceContainerNode::init()
{
    this->loadNode_srv_.reset();
    this->unloadNode_srv_.reset();
    if (!this->get_parameter("can_interface_name", can_interface_name_))
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter can_interface_name could not be read.");
    }
    if (!this->get_parameter("master_config", dcf_txt_))
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter master_config could not be read.");
    }
    if (!this->get_parameter("master_bin", dcf_bin_))
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter master_bin could not be read.");
    }

    if (!this->get_parameter("bus_config", bus_config_))
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter bus_config could not be read.");
    }

    RCLCPP_INFO(this->get_logger(), "Starting Device Container with:");
    RCLCPP_INFO(this->get_logger(), "\t master_config %s", dcf_txt_.c_str());
    RCLCPP_INFO(this->get_logger(), "\t bus_config %s", bus_config_.c_str());

    this->config_ = std::make_shared<ros2_canopen::ConfigurationManager>(bus_config_);
    this->config_->init_config();

    if (!this->load_master_from_config())
    {
        return false;
    }
    if (!this->load_drivers_from_config())
    {
        return false;
    }

    if (!this->load_manager())
    {
        return false;
    }

    return true;
}

void LifecycleDeviceContainerNode::on_list_nodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response)
{
    auto components = list_components();
    // RCLCPP_INFO(this->get_logger(), "List of active components:");
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        // RCLCPP_INFO(this->get_logger(), "%i : %s", it->first, it->second.c_str());
        response->unique_ids.push_back(it->first);
        response->full_node_names.push_back(it->second);
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto lifecycle_device_container_node = std::make_shared<LifecycleDeviceContainerNode>(exec);
    exec->add_node(lifecycle_device_container_node);
    std::thread spinThread([&exec]() {
        exec->spin();
    });
    std::this_thread::sleep_for(100ms);
    if (lifecycle_device_container_node->init())
    {
        RCLCPP_INFO(lifecycle_device_container_node->get_logger(), "Initialisation successful.");
    }
    else
    {
        RCLCPP_INFO(lifecycle_device_container_node->get_logger(), "Initialisation failed.");
    }

    spinThread.join();
    return 0;
}
