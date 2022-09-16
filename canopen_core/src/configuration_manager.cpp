#include "canopen_core/configuration_manager.hpp"
#include <stdexcept>

namespace ros2_canopen
{

    void ConfigurationManager::init_config()
    {
        for (
            YAML::const_iterator it = root_.begin();
            it != root_.end();
            it++)
        {
            std::string driver_name = it->first.as<std::string>();
            YAML::Node config_node = it->second;
            devices_.insert({driver_name, config_node});
        }
    }

    uint32_t ConfigurationManager::get_all_devices(std::vector<std::string> &devices)
    {
        uint32_t count = 0;
        for (auto it = devices_.begin(); it != devices_.end(); it++)
        {
            devices.emplace_back(it->first);
            count++;
        }
        return count;
    }

} // namespace ros2_canopen
