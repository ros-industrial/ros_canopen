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
#include <lely/ev/loop.hpp>
#if _WIN32
#include <lely/io2/win32/ixxat.hpp>
#include <lely/io2/win32/poll.hpp>
#elif defined(__linux__)
#include <lely/io2/posix/poll.hpp>
#else
#error This file requires Windows or Linux.
#endif
#include <yaml-cpp/yaml.h>
#include "ros2_canopen_core/device_manager.hpp"

using namespace ros2_canopen;


void
DeviceManager::set_executor(const std::weak_ptr<rclcpp::Executor> executor)
{
  executor_ = executor;
}

void
DeviceManager::add_node_to_executor(const std::string &plugin_name, const uint32_t node_id, const std::string &node_name)
{
    RCLCPP_INFO(this->get_logger(), "Initialising loaded %s", plugin_name.c_str());
    if (auto exec = executor_.lock()) {
        exec->add_node(node_wrappers_[node_id].get_node_base_interface(), true);

        auto node_instance = std::static_pointer_cast<ros2_canopen::DriverInterface>(node_wrappers_[node_id].get_node_instance());

        // maybe this is not necessary anymore since node_wrappers_ is storing the wrappers
        drivers_.insert({node_id, node_instance});

        can_master_->add_driver(node_instance, node_id);

        RCLCPP_INFO(this->get_logger(), "Added %s of type %s to executor", node_name.c_str(), plugin_name.c_str());
    } else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to add %s of type %s to executor", node_name.c_str(), plugin_name.c_str());
    }
}

void
DeviceManager::remove_node_from_executor(const std::string &plugin_name, const uint32_t node_id, const std::string &node_name)
{
    RCLCPP_INFO(this->get_logger(), "Removing %s", plugin_name.c_str());
    if (auto exec = executor_.lock()) {
        exec->remove_node(node_wrappers_[node_id].get_node_base_interface());

        auto node_instance = std::static_pointer_cast<ros2_canopen::DriverInterface>(node_wrappers_[node_id].get_node_instance());
        can_master_->remove_driver(node_instance, node_id);

        RCLCPP_INFO(this->get_logger(), "Removed %s of type %s from executor", node_name.c_str(), plugin_name.c_str());
    } else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to remove %s of type %s from executor", node_name.c_str(), plugin_name.c_str());
    }
}

bool DeviceManager::load_component(const std::string &pkg_name, const std::string &plugin_name, uint32_t node_id, std::string &node_name)
{
    ComponentResource component;
    std::vector<ComponentResource> components = this->get_component_resources(pkg_name);
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        if (it->first.compare(plugin_name) == 0)
        {
            auto factory_node = this->create_component_factory(*it);
            rclcpp::NodeOptions opts;
            opts.use_global_arguments(false);
            std::vector<std::string> remap_rules;
            remap_rules.push_back("--ros-args");
            remap_rules.push_back("-r");
            remap_rules.push_back("__node:=" + node_name);
            opts.arguments(remap_rules);

            try {
                node_wrappers_[node_id] = factory_node->create_node_instance(opts);
                add_node_to_executor(plugin_name, node_id, node_name);
            } catch (const std::exception & ex) {
                // In the case that the component constructor throws an exception,
                // rethrow into the following catch block.
                throw rclcpp_components::ComponentManagerException(
                        "Component constructor threw an exception: " + std::string(ex.what()));
            } catch (...) {
                // In the case that the component constructor throws an exception,
                // rethrow into the following catch block.
                throw rclcpp_components::ComponentManagerException("Component constructor threw an exception");
            }

            return true;
        }
    }
    return false;
}

std::map<uint32_t, std::string> DeviceManager::list_components()
{
    std::map<uint32_t, std::string> components;
    for (auto & wrapper : node_wrappers_)
    {
        components[wrapper.first] =
            wrapper.second.get_node_base_interface()->get_fully_qualified_name();
    }
    return components;
}

bool DeviceManager::init_devices_from_config(
    std::string &dcf_txt,
    std::string &bus_config,
    std::string &dcf_bin,
    std::string &can_interface_name)
{

    YAML::Node node;
    try
    {
        node = YAML::LoadFile(bus_config.c_str());
    }
    catch (const YAML::BadFile &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return false;
    }

    for (
        YAML::const_iterator it = node.begin();
        it != node.end();
        it++)
    {
        // Get toplevel node name
        std::string driver_name = it->first.as<std::string>();
        // Device config
        YAML::Node config = it->second;
        int node_id = config["node_id"].as<int>();

        // init master
        if (driver_name.find("master") != std::string::npos)
        {
            // TODO: load master component
            try
            {
                can_master_ =
                    std::make_shared<ros2_canopen::MasterNode>(
                        "master",
                        rclcpp::NodeOptions().use_global_arguments(false),
                        dcf_txt,
                        dcf_bin,
                        can_interface_name,
                        node_id);
                if (auto exec = executor_.lock())
                {
                    exec->add_node(can_master_->get_node_base_interface(), true);
                    RCLCPP_INFO(this->get_logger(), "Added master to executor");
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        else
        {
            // load the driver
            std::string plugin_name = config["driver"].as<std::string>();
            std::string package_name = config["package"].as<std::string>();
            // TODO: if one of the driver fails to load,
            //  should the state change or exit with FAILURE?

            this->load_component(package_name, plugin_name, node_id, driver_name);
        }
    }
    auto components = list_components();
    RCLCPP_INFO(this->get_logger(), "List of active components:");
    for(auto it = components.begin(); it != components.end(); ++it)
    {
        RCLCPP_INFO(this->get_logger(), "%i : %s", it->first, it->second.c_str());
    }
    return true;
}

bool DeviceManager::init()
{
    std::string can_interface_name;
    std::string dcf_txt;
    std::string bus_config;
    std::string dcf_bin;

    this->get_parameter("can_interface_name", can_interface_name);
    this->get_parameter("master_config", dcf_txt);
    this->get_parameter("master_bin", dcf_bin);
    this->get_parameter("bus_config", bus_config);

    init_devices_from_config(dcf_txt, bus_config, dcf_bin, can_interface_name);
    return true;
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto device_manager = std::make_shared<DeviceManager>(exec);
    std::thread spinThread([&device_manager]()
                           { std::cout << "Init success: " << device_manager->init() << std::endl; });
    exec->add_node(device_manager);
    exec->spin();
    spinThread.join();
    return 0;
}
