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

#ifndef DEVICE_HPP_
#define DEVICE_HPP_

#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace lely;

namespace ros2_canopen
{
    // Base class for driver plugin
    // Pluginlib API does allows only default constructors
    class DriverInterface : public rclcpp::Node
    {
    public:
        /**
         * @brief Construct a new DriverInterface object
         *
         * @param [in] node_name
         * @param [in] node_options
         */
        DriverInterface(const std::string &node_name,
                             const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, node_options) {}

        /**
         * @brief Init driver with Master
         *
         * @param [in] exec         Executor that the driver will be added to
         * @param [in] master       Master that controls the driver
         * @param [in] node_id      Node ID of the driver
         */
        virtual void init(ev::Executor &exec,
                          canopen::AsyncMaster &master,
                          uint8_t node_id) noexcept = 0;

        /**
         * @brief Remove driver from Master
         *
         * @param [in] exec               
         * @param [in] master
         * @param [in] node_id
         */
        virtual void remove(ev::Executor &exec,
                            canopen::AsyncMaster &master,
                            uint8_t node_id) noexcept = 0;
    };

    class MasterDevice : public rclcpp::Node
    {
    protected:
        std::string dcf_txt_;
        std::string dcf_bin_;
        std::string can_interface_name_;
        uint8_t node_id_;

    public:
        MasterDevice(
            const std::string &node_name,
            const rclcpp::NodeOptions &node_options,
            std::string dcf_txt,
            std::string dcf_bin,
            std::string can_interface_name,
            uint8_t nodeid) : rclcpp::Node(node_name, node_options)
        {
            dcf_txt_ = dcf_txt;
            dcf_bin_ = dcf_bin;
            can_interface_name_ = can_interface_name;
            node_id_ = nodeid;
        }

        virtual void add_driver(std::shared_ptr<ros2_canopen::DriverInterface>, uint8_t node_id) = 0;
        virtual void remove_driver(std::shared_ptr<ros2_canopen::DriverInterface>, uint8_t node_id) = 0;
    };

    class SlaveDevice : public rclcpp::Node, public canopen::BasicSlave
    {
    public:
        SlaveDevice(io::Timer &timer,
                    io::CanChannel &chan,
                    std::string &dcf_txt,
                    u_int32_t id,
                    const std::string &devName)
            : rclcpp::Node(devName),
              canopen::BasicSlave(timer, chan, dcf_txt, "", id) {}

        // ROS interfaces
    };
} // end ros2_canopen namespace

#endif // DEVICE_HPP_