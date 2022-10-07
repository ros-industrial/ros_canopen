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

#include "canopen_core/device_container.hpp"
#include "canopen_core/device_container_error.hpp"

using namespace ros2_canopen;

void DeviceContainer::set_executor(const std::weak_ptr<rclcpp::Executor> executor)
{
    executor_ = executor;
}

bool DeviceContainer::init_driver(uint16_t node_id)
{
    RCLCPP_DEBUG(this->get_logger(), "init_driver");
    registered_drivers_[node_id]->set_master(this->can_master_->get_executor(), this->can_master_->get_master());
    return true;
}

bool DeviceContainer::load_component(
    std::string &package_name,
    std::string &driver_name,
    uint16_t node_id,
    std::string &node_name,
    std::vector<rclcpp::Parameter> &params)
{
    ComponentResource component;
    std::string resource_index("rclcpp_components");
    std::vector<ComponentResource> components = this->get_component_resources(package_name, resource_index);
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        if (it->first.compare(driver_name) == 0)
        {
            std::shared_ptr<rclcpp_components::NodeFactory> factory_node = this->create_component_factory(*it);
            rclcpp::NodeOptions opts;
            opts.use_global_arguments(false);
            std::vector<std::string> remap_rules;
            remap_rules.push_back("--ros-args");
            //remap_rules.push_back("--log-level");
            //remap_rules.push_back("debug");
            remap_rules.push_back("-r");
            remap_rules.push_back("__node:=" + node_name);

            opts.arguments(remap_rules);
            opts.parameter_overrides(params);

            try
            {
                auto wrapper = factory_node->create_node_instance(opts);
                if (node_name.compare("master") == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Load master component.");
                    can_master_ =
                        std::static_pointer_cast<CanopenMasterInterface>(wrapper.get_node_instance());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Load driver component.");
                    if (this->lifecycle_operation_)
                    {
                        auto node = std::static_pointer_cast<CanopenDriverInterface>(wrapper.get_node_instance());
                        if (!node->is_lifecycle())
                        {
                            std::string execption_string;
                            execption_string.append("Driver ");
                            execption_string.append(driver_name);
                            execption_string.append(" for device ");
                            execption_string.append(node_name);
                            execption_string.append(" is not a lifecycle driver while the master is.");
                            RCLCPP_ERROR(this->get_logger(), execption_string.c_str());
                            throw DeviceContainerException(execption_string);
                        }
                        else
                        {
                            registered_drivers_[node_id] = node;
                        }
                    }
                    else
                    {
                        auto node = std::static_pointer_cast<CanopenDriverInterface>(wrapper.get_node_instance());
                        if (node->is_lifecycle())
                        {
                            std::string execption_string;
                            execption_string.append("Driver ");
                            execption_string.append(driver_name);
                            execption_string.append(" for device ");
                            execption_string.append(node_name);
                            execption_string.append(" is a lifecycle driver while the master is not.");
                            RCLCPP_ERROR(this->get_logger(), execption_string.c_str());
                            throw DeviceContainerException(execption_string);
                        }
                        else
                        {
                            registered_drivers_[node_id] = node;
                        }
                    }
                }
            }
            catch (const std::exception &ex)
            {
                // In the case that the component constructor throws an exception,
                // rethrow into the following catch block.
                throw DeviceContainerException(
                    "Component constructor threw an exception: " + std::string(ex.what()));
            }
            catch (...)
            {
                // In the case that the component constructor throws an exception,
                // rethrow into the following catch block.
                throw DeviceContainerException("Component constructor threw an exception");
            }

            return true;
        }
    }
    return false;
}

void DeviceContainer::configure()
{
    if (!this->get_parameter("can_interface_name", can_interface_name_))
    {
        throw DeviceContainerException("Fatal: Getting Parameter failed.");
        RCLCPP_ERROR(this->get_logger(), "Parameter can_interface_name could not be read.");
    }
    if (!this->get_parameter("master_config", dcf_txt_))
    {
        throw DeviceContainerException("Fatal: Getting Parameter failed.");
        RCLCPP_ERROR(this->get_logger(), "Parameter master_config could not be read.");
    }
    if (!this->get_parameter("master_bin", dcf_bin_))
    {
        throw DeviceContainerException("Fatal: Getting Parameter failed.");
        RCLCPP_ERROR(this->get_logger(), "Parameter master_bin could not be read.");
    }

    if (!this->get_parameter("bus_config", bus_config_))
    {
        throw DeviceContainerException("Fatal: Getting Parameter failed.");
        RCLCPP_ERROR(this->get_logger(), "Parameter bus_config could not be read.");
    }

    if(can_interface_name_.length() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter can_interface_name was not set.");
        throw DeviceContainerException("Fatal: Getting Parameter failed.");
    }

    if(dcf_txt_.length() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter master_config was not set.");
        throw DeviceContainerException("Fatal: Getting Parameter failed.");
    }

    if(bus_config_.length() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter bus_config was not set.");
        throw DeviceContainerException("Fatal: Getting Parameter failed.");
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting Device Container with:");
    RCLCPP_INFO(this->get_logger(), "\t master_config %s", dcf_txt_.c_str());
    RCLCPP_INFO(this->get_logger(), "\t bus_config %s", bus_config_.c_str());
    RCLCPP_INFO(this->get_logger(), "\t can_interface_name %s", can_interface_name_.c_str());

    try
    {
        this->config_ = std::make_unique<ros2_canopen::ConfigurationManager>(bus_config_);
        this->config_->init_config();

    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Loading bus configuration %s failed", bus_config_.c_str());
        throw DeviceContainerException("Fatal: Loading bus configuration " + bus_config_ + " failed.");
    }
}

bool DeviceContainer::load_master()
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
            std::vector<rclcpp::Parameter> params;
            params.push_back(rclcpp::Parameter("container_name", this->get_fully_qualified_name()));
            params.push_back(rclcpp::Parameter("master_dcf", this->dcf_txt_));
            params.push_back(rclcpp::Parameter("master_bin", this->dcf_bin_));
            params.push_back(rclcpp::Parameter("can_interface_name", this->can_interface_name_));
            params.push_back(rclcpp::Parameter("node_id", (int)node_id.value()));
            params.push_back(rclcpp::Parameter("non_transmit_timeout", 100));
            params.push_back(rclcpp::Parameter("config", config_->dump_device(*it)));

            if (!this->load_component(package_name.value(), driver_name.value(), node_id.value(), *it, params))
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Loading master failed.");
                return false;
            }

            add_node_to_executor(can_master_->get_node_base_interface());
            can_master_->init();
            master_found = true;
            can_master_id_ = node_id.value();
            this->lifecycle_operation_ = can_master_->is_lifecycle();
        }
    }

    if (!master_found)
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Master not in configuration");
        return false;
    }
    return true;
}

bool DeviceContainer::load_drivers()
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

            if (node_id.value() == can_master_id_)
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has duplicate entry for node id %i", node_id.value());
                return false;
            }

            if (registered_drivers_.count(node_id.value()) != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has duplicate entry for node id %i", node_id.value());
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "Found device %s with driver %s", it->c_str(), driver_name.value().c_str());
            
            std::vector<rclcpp::Parameter> params;
            params.push_back(rclcpp::Parameter("container_name", this->get_fully_qualified_name()));
            params.push_back(rclcpp::Parameter("node_id", (int)node_id.value()));
            params.push_back(rclcpp::Parameter("config", config_->dump_device(*it)));
            params.push_back(rclcpp::Parameter("non_transmit_timeout", 100));

            if (!this->load_component(package_name.value(), driver_name.value(), node_id.value(), *it, params))
            {
                RCLCPP_ERROR(this->get_logger(), "Loading driver failed: node_id(%hu), node_name(%s), driver(%s), driver_package(%s)",
                    node_id.value(),
                    it->c_str(),
                    driver_name.value().c_str(),
                    package_name.value().c_str());
                return false;
            }
            add_node_to_executor(registered_drivers_[node_id.value()]->get_node_base_interface());
            registered_drivers_[node_id.value()]->init();
        }
    }
    return true;
}

bool DeviceContainer::load_manager()
{
    if(this->lifecycle_operation_)
    {
        RCLCPP_INFO(this->get_logger(), "Loading Manager Configuration.");
        auto node_options = rclcpp::NodeOptions();
        node_options.use_global_arguments(false);

        //Set parameters
        rclcpp::Parameter container_name("container_name", this->get_fully_qualified_name());
        std::vector<rclcpp::Parameter> params;
        params.push_back(container_name);
        node_options.parameter_overrides(params);

        //Instantiate Manager
        this->lifecycle_manager_ = std::make_unique<ros2_canopen::LifecycleManager>(
            node_options
        );

        //Add to executor
        add_node_to_executor(this->lifecycle_manager_->get_node_base_interface());

        //Initialise based on configuration file.
        this->lifecycle_manager_->init(this->config_);
    }
    return true;
}

void DeviceContainer::init()
{
    configure();
    if (!this->load_master())
    {
        throw DeviceContainerException("Fatal: Loading Master Failed.");
    }
    if (!this->load_drivers())
    {
        throw DeviceContainerException("Fatal: Loading Drivers Failed.");
    }

    if (!this->load_manager())
    {
        throw DeviceContainerException("Fatal: Loading Manager Failed.");
    }
}

void DeviceContainer::init(
    const std::string &can_interface_name,
    const std::string &master_config,
    const std::string &bus_config,
    const std::string &master_bin)
{
    can_interface_name_ = can_interface_name;
    dcf_txt_ = master_config;
    dcf_bin_ = master_bin;
    bus_config_ = bus_config;

    RCLCPP_INFO(this->get_logger(), "Starting Device Container with:");
    RCLCPP_INFO(this->get_logger(), "\t master_config %s", dcf_txt_.c_str());
    RCLCPP_INFO(this->get_logger(), "\t bus_config %s", bus_config_.c_str());

    this->config_ = std::make_unique<ros2_canopen::ConfigurationManager>(bus_config_);
    this->config_->init_config();

    if (!this->load_master())
    {
        throw DeviceContainerException("Fatal: Loading Master Failed.");
    }
    if (!this->load_drivers())
    {
        throw DeviceContainerException("Fatal: Loading Drivers Failed.");
    }

    if (!this->load_manager())
    {
        throw DeviceContainerException("Fatal: Loading Manager Failed.");
    }
}

void DeviceContainer::on_list_nodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response)
{
    auto components = list_components();
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        RCLCPP_INFO(this->get_logger(), "%hu - %s", it->first, it->second.c_str());
        response->unique_ids.push_back((uint64_t)it->first);
        response->full_node_names.push_back(it->second);
    }
}

std::map<uint16_t, std::string> DeviceContainer::list_components()
{
    std::map<uint16_t, std::string> components;

    components[can_master_id_] =
        can_master_->get_node_base_interface()->get_fully_qualified_name();

    if(this->lifecycle_operation_)
    {
        components[256] =
            lifecycle_manager_->get_node_base_interface()->get_fully_qualified_name();
    }

    for (auto &driver : registered_drivers_)
    {
        components[driver.first] =
            driver.second->get_node_base_interface()->get_fully_qualified_name();
    }

    return components;
}

// int main(int argc, char const *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
//     auto lifecycle_device_container_node = std::make_shared<DeviceContainer>(exec);
//     exec->add_node(lifecycle_device_container_node);
//     std::thread spinThread([&exec]()
//                            { exec->spin(); });
//     std::this_thread::sleep_for(100ms);
//     if (lifecycle_device_container_node->init())
//     {
//         RCLCPP_INFO(lifecycle_device_container_node->get_logger(), "Initialisation successful.");
//     }
//     else
//     {
//         RCLCPP_INFO(lifecycle_device_container_node->get_logger(), "Initialisation failed.");
//     }

//     spinThread.join();
//     return 0;
// }

// bool DeviceContainer::set_remote_paramter(std::string node_name, rclcpp::Parameter &param)
// {
//     std::string service_name = node_name + "/set_parameters";
//     rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameter_client;

//     set_parameter_client = node_->create_client<rcl_interfaces::srv::SetParameters>(
//         service_name);

//     while (!set_parameter_client->wait_for_service(non_transmit_timeout_))
//     {
//         if (!rclcpp::ok())
//         {
//             RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for init_driver service. Exiting.");
//         }
//         RCLCPP_INFO(node_->get_logger(), "init_driver service not available, waiting again...");
//     }
//     auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
//     request->parameters.push_back(param.to_parameter_msg());

//     auto future_result = demand_set_master_client->async_send_request(request);

//     auto future_status = future_result.wait_for(std::chrono::milliseconds(1000));
//     RCLCPP_DEBUG(node_->get_logger(), "demand_set_master end");

//     if (future_status == std::future_status::ready)
//     {
//         future_result.get();
//     }
// }
