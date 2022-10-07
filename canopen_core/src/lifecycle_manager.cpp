#include "canopen_core/lifecycle_manager.hpp"

namespace ros2_canopen
{

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleManager::on_configure(const rclcpp_lifecycle::State &state)
    {
        this->get_parameter<std::string>("container_name", this->container_name_);

        bool res = this->load_from_config();
        if (!res)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load from config");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleManager::on_activate(const rclcpp_lifecycle::State &state)
    {
        if (!this->bring_up_all())
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleManager::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        if (!this->bring_down_all())
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleManager::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleManager::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void
    LifecycleManager::init(std::shared_ptr<ros2_canopen::ConfigurationManager> config)
    {
        this->config_ = config;
    }

    bool
    LifecycleManager::load_from_config()
    {

        std::vector<std::string> devices;
        uint32_t count = this->config_->get_all_devices(devices);
        RCLCPP_INFO(this->get_logger(), "Configuring for %u devices.", count);

        // Find master in configuration
        for (auto it = devices.begin(); it != devices.end(); it++)
        {
            uint8_t node_id = config_->get_entry<uint8_t>(*it, "node_id").value();
            std::string change_state_client_name = *it;
            std::string get_state_client_name = *it;
            get_state_client_name += "/get_state";
            change_state_client_name += "/change_state";
            RCLCPP_INFO(this->get_logger(), "Found %s (node_id=%hu)", it->c_str(), node_id);
            device_names_to_ids.emplace(*it, node_id);
            rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client =
                this->create_client<lifecycle_msgs::srv::GetState>(get_state_client_name, rmw_qos_profile_services_default, cbg_clients);

            rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client =
                this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_client_name, rmw_qos_profile_services_default, cbg_clients);

            this->drivers_get_state_clients.emplace(node_id, get_state_client);
            this->drivers_change_state_clients.emplace(node_id, change_state_client);

            if (it->find("master") != std::string::npos)
            {
                this->master_id_ = node_id;
            }
        }
        return true;
    }

    unsigned int
    LifecycleManager::get_state(uint8_t node_id, std::chrono::seconds time_out)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto client = this->drivers_get_state_clients[node_id];
        if (!client->wait_for_service(time_out))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Service %s is not available.",
                client->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        // We send the service request for asking the current
        // state of the lc_talker node.
        auto future_result = client->async_send_request(request);

        // Let's wait until we have the answer from the node.
        // If the request times out, we return an unknown state.
        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(), "Server time out while getting current state for node %hhu", node_id);
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
        auto result = future_result.get()->current_state;
        return result.id;
    }

    bool
    LifecycleManager::change_state(uint8_t node_id, uint8_t transition, std::chrono::seconds time_out)
    {
        auto client = this->drivers_change_state_clients[node_id];
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        if (!client->wait_for_service(time_out))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Service %s is not available.",
                client->get_service_name());
            return false;
        }

        // We send the request with the transition we want to invoke.
        auto future_result = client->async_send_request(request);

        // Let's wait until we have the answer from the node.
        // If the request times out, we return an unknown state.
        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(), "Server time out while getting current state for node %hhu", node_id);
            return false;
        }

        if (future_result.get()->success)
        {
            return true;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
            return false;
        }
        return false;
    }

    bool
    LifecycleManager::bring_up_master()
    {
        auto state = this->get_state(master_id_, 3s);
        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up master. Master not in unconfigured state.");
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "Master (node_id=%hu) has state unconfigured.", master_id_);
        if (!this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 3s))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up master. Configure Transition failed.");
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "Master (node_id=%hu) has state inactive.", master_id_);
        if (!this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 3s))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up master. Activate Transition failed.");
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "Master (node_id=%hu) has state active.", master_id_);
        return true;
    }

    bool
    LifecycleManager::bring_down_master()
    {
        this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 3s);
        this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 3s);

        auto state = this->get_state(master_id_, 3s);

        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
            return false;
        }

        return true;
    }

    bool
    LifecycleManager::bring_up_driver(std::string device_name)
    {
        
        auto node_id = this->device_names_to_ids[device_name];
        RCLCPP_DEBUG(this->get_logger(), "Bringing up %s with id %u", device_name.c_str(), node_id);
        auto master_state = this->get_state(master_id_, 3s);
        if (master_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s. Master not in active state.", device_name.c_str());
            return false;
        }
        auto state = this->get_state(node_id, 3s);
        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s. Not in unconfigured state.", device_name.c_str());
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "%s (node_id=%hu) has state unconfigured. Attempting to configure.", device_name.c_str(), node_id);
        if (!this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 3s))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s. Configure Transition failed.", device_name.c_str());
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "%s (node_id=%hu) has state inactive. Attempting to activate.", device_name.c_str(), node_id);
        if (!this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 3s))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s. Activate Transition failed.", device_name.c_str());
            return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "%s (node_id=%hu) has state active.", device_name.c_str(), node_id);
        return true;
    }

    bool
    LifecycleManager::bring_down_driver(std::string device_name)
    {
        auto node_id = this->device_names_to_ids[device_name];

        this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 3s);
        this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 3s);
        auto state = this->get_state(node_id, 3s);
        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
            return false;
        }
        return true;
    }

    bool
    LifecycleManager::bring_up_all()
    {
        if (!this->bring_up_master())
        {
            return false;
        }
        for (auto it = this->device_names_to_ids.begin(); it != this->device_names_to_ids.end(); ++it)
        {
            if (it->first.find("master") == std::string::npos)
            {
                if (!this->bring_up_driver(it->first))
                {
                    return false;
                }
            }
            else{
                RCLCPP_DEBUG(this->get_logger(), "Skipped master.");
            }
        }
        return true;
    }

    bool
    LifecycleManager::bring_down_all()
    {
        RCLCPP_INFO(this->get_logger(), "Bring Down all");
        for (auto it = this->device_names_to_ids.begin(); it != this->device_names_to_ids.end(); ++it)
        {
            if (it->first.compare("master") != 0)
            {
                if (!this->bring_down_driver(it->first))
                {
                    return false;
                }
            }
        }
        if (!this->bring_down_master())
        {
            return false;
        }

        return true;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleManager)