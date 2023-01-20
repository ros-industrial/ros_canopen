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

#ifndef CONFIGURATION_MANAGER_HPP
#define CONFIGURATION_MANAGER_HPP

#include <iostream>
#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

namespace ros2_canopen
{
/**
 * @brief Manager for Bus Configuration.
 *
 * The Bus configuration Manager stores the YAML bus configuration and
 * enables reading configuration entries. The configuration manager is passed
 * to all ros2_canopen master and slave drivers to enable reading driver specific
 * configuration parameters from the YAML configuration file.
 *
 */
class ConfigurationManager
{
private:
  std::string file_;                           ///< Stores the configuration file name
  YAML::Node root_;                            ///< Stores YAML root node
  std::map<std::string, YAML::Node> devices_;  ///< Stores all configuration per device

public:
  ConfigurationManager(std::string & file) : file_(file) { root_ = YAML::LoadFile(file_.c_str()); }

  /**
   * @brief Gets a configuration entry for a specific device
   *
   * @tparam T                Datatype of the retrieved object
   * @param device_name       Device name
   * @param entry_name        Entry name
   * @return std::optional<T> Return value, can be empty.
   */
  template <typename T>
  std::optional<T> get_entry(std::string device_name, std::string entry_name)
  {
    try
    {
      auto config = devices_.at(device_name);
      return std::optional<T>(config[entry_name.c_str()].as<T>());
    }
    catch (const std::exception & e)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("yaml-cpp"), "Failed to load entry \"%s\" for device \"%s\" ",
        entry_name.c_str(), device_name.c_str());
    }

    return std::nullopt;
  }

  /**
   * @brief Dump device string
   *
   * @param device_name
   * @return std::string
   */
  std::string dump_device(std::string device_name)
  {
    std::string result;
    try
    {
      auto config = devices_.at(device_name);
      result = YAML::Dump(config);
    }
    catch (const std::exception & e)
    {
      std::cerr << e.what() << '\n';
    }
    return result;
  }

  /**
   * @brief Initialises the configuration.
   *
   */
  void init_config();

  /**
   * @brief Returns all device names
   *
   * @param devices           List with names of all devices
   * @return uint32_t         Number of devices discovered
   */
  uint32_t get_all_devices(std::vector<std::string> & devices);
};
}  // namespace ros2_canopen

#endif
