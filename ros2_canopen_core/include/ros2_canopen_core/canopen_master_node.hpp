//    Copyright 2022 Christoph Hellmann Santos
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
#ifndef ROS2_CANOPEN_NODE_HPP
#define ROS2_CANOPEN_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <utility>
#include <fstream>
#include <map>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/ev/co_task.hpp>

#include <yaml-cpp/yaml.h>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "pluginlib/class_loader.hpp"

#include "ros2_canopen_interfaces/srv/co_nmt_id.hpp"
#include "ros2_canopen_interfaces/srv/co_read_id.hpp"
#include "ros2_canopen_interfaces/srv/co_heartbeat_id.hpp"
#include "ros2_canopen_interfaces/srv/co_write_id.hpp"
#include "ros2_canopen_core/proxy_device.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace lely;

namespace ros2_canopen
{
    /**
     * @brief CANopen Master Node
     *
     * This class provides a CANopen Master Node.
     *
     */
    class CANopenNode : public rclcpp_lifecycle::LifecycleNode
    {
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std::shared_ptr<io::IoGuard> io_guard;
        std::shared_ptr<io::Context> ctx;
        std::shared_ptr<io::Poll> poll;
        std::shared_ptr<ev::Loop> loop;
        std::shared_ptr<ev::Executor> exec;
        std::shared_ptr<io::CanController> ctrl;
        std::shared_ptr<io::CanChannel> chan;
        std::shared_ptr<io::Timer> can_timer;
        std::shared_ptr<canopen::AsyncMaster> can_master;
        std::shared_ptr<ros2_canopen::ProxyDevice> basicdevice;
        std::shared_ptr<std::string> nmt_status;

        std::string can_interface_name;
        std::string dcf_path;
        std::string yaml_path;
        std::map<int, std::string> drivers;
        std::shared_ptr<std::map<int, std::shared_ptr<ros2_canopen::CANopenDevice>>> devices;
        std::unique_ptr<std::thread> canopen_loop_thread;

        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::CONmtID>> master_nmt_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::COReadID>> master_read_sdo_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::COHeartbeatID>> master_set_hearbeat_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::COWriteID>> master_write_sdo_service;

        std::atomic<bool> active;
        std::atomic<bool> configured;
        std::shared_ptr<std::mutex> master_mutex;

        std::promise<void> activation_done;
        std::promise<void> configuration_done;
        std::promise<void> activation_started;
        std::promise<void> cleanup_done;
        std::future<void> master_thread_running;

        // Service Callback Declarations
        void master_nmt(
            const std::shared_ptr<ros2_canopen_interfaces::srv::CONmtID::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::CONmtID::Response> response);

        void master_read_sdo(
            const std::shared_ptr<ros2_canopen_interfaces::srv::COReadID::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::COReadID::Response> response);

        void master_set_heartbeat(
            const std::shared_ptr<ros2_canopen_interfaces::srv::COHeartbeatID::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::COHeartbeatID::Response> response);

        void master_write_sdo(
            const std::shared_ptr<ros2_canopen_interfaces::srv::COWriteID::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::COWriteID::Response> response);

        // helper functions
        void run();
        void read_yaml();
        void register_services();
        void register_drivers();
        void deregister_drivers();
        CallbackReturn change_state(
            const std::uint8_t transition,
            std::chrono::seconds time_out = 1s);

        // Tasks


        std::promise<uint32_t> sdo_read_data_promise;
        void read_callback(uint8_t id, uint16_t idx, uint8_t subidx,
                            ::std::error_code ec, uint32_t value) 
        {
            if (ec)
                this->sdo_read_data_promise.set_exception(lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "Master Write SDO"));
            else
            {
                this->sdo_read_data_promise.set_value((uint32_t) value);
            }
        }

        template <typename T>
        void master_read(
            const std::shared_ptr<ros2_canopen_interfaces::srv::COReadID::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::COReadID::Response> response)
        {
            if (active.load())
            {
                sdo_read_data_promise = std::promise<uint32_t>();
                this->can_master->SubmitRead<T>(
                    *(this->exec),
                    request->nodeid, 
                    request->index, 
                    request->subindex,
                    std::bind(&CANopenNode::read_callback, this, 
                        std::placeholders::_1, 
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4,
                        std::placeholders::_5
                        )
                    );
                auto f = sdo_read_data_promise.get_future();
                f.wait();
                try
                {
                    response->data = f.get();
                    response->success = true;
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), e.what());
                    response->success = false;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Couldn't write SDO because node not active");
                response->success = false;
            }
        }

        std::promise<void> sdo_write_data_promise;
        void write_callback(uint8_t id, uint16_t idx, uint8_t subidx,
                            ::std::error_code ec) 
        {
            if (ec)
                this->sdo_write_data_promise.set_exception(lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "Master Write SDO"));
            else
            {
                this->sdo_write_data_promise.set_value();
            }
        }

        template <typename T>
        void master_write(
            const std::shared_ptr<ros2_canopen_interfaces::srv::COWriteID::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::COWriteID::Response> response)
        {
            if (active.load())
            { 
                sdo_write_data_promise = std::promise<void>();
                this->can_master->SubmitWrite(
                    *(this->exec),
                    request->nodeid, 
                    request->index, 
                    request->subindex, 
                    static_cast<T>(request->data), 
                    std::bind(&CANopenNode::write_callback, this, 
                        std::placeholders::_1, 
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4
                        )
                    );
                auto f = sdo_write_data_promise.get_future();
                f.wait();
                try
                {
                    f.get();
                    response->success = true;
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), e.what());
                    response->success = false;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Couldn't write SDO because node not active");
                response->success = false;
            }
        }
        // Lifecycle Callback Functions
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
        pluginlib::ClassLoader<ros2_canopen::CANopenDevice> poly_loader;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        rclcpp::CallbackGroup::SharedPtr lifecycle_manager_group;

    public:
        /**
         * @brief Construct a new CANopenNode object
         *
         * @param node_name
         * @param intra_process_comms
         */
        CANopenNode(const std::string &node_name, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(
                  node_name,
                  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
              poly_loader("ros2_canopen_core", "ros2_canopen::CANopenDevice"),
              executor_(executor)
        {
            this->declare_parameter<std::string>("can_interface_name", "vcan0");
            this->declare_parameter<std::string>("dcf_path", "");
            this->declare_parameter<std::string>("yaml_path", "");
            can_interface_name = "";
            dcf_path = "";
            yaml_path = "";
            this->register_services();
            this->master_mutex = std::make_shared<std::mutex>();
            this->devices = std::make_shared<std::map<int, std::shared_ptr<ros2_canopen::CANopenDevice>>>();
            lifecycle_manager_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        }
    };
}

#endif