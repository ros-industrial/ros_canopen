#ifndef BASIC_DEVICE_HPP
#define BASIC_DEVICE_HPP
#include <memory>
#include <mutex>
#include <atomic>
#include <future>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>

#include "canopen_device_base.hpp"
#include "ros2_canopen_interfaces/msg/pdo.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_canopen
{

    struct PDOData
    {
    public:
        uint16_t index_;
        uint8_t subindex_;
        uint32_t data_;
        PDOData(
            uint16_t index,
            uint8_t subindex,
            uint32_t data) : index_(index), subindex_(subindex), data_(data)
        {
        }

        PDOData(
            const PDOData &obj) : index_(obj.index_), subindex_(obj.subindex_), data_(obj.data_)
        {
        }
    };
    class BasicDeviceSharedData
    {
    private:
        std::promise<canopen::NmtState> nmt_state_promise;
        std::atomic<bool> nmt_state_is_set;

        std::promise<ros2_canopen::PDOData> rpdo_promise;
        std::atomic<bool> rpdo_is_set;
        std::mutex nmt_mtex;
        std::mutex pdo_mtex;

    public:
        std::shared_ptr<std::mutex> master_mutex;
        std::shared_ptr<canopen::AsyncMaster> master;
        std::shared_ptr<ev::Executor> exec;
        uint8_t id;
        BasicDeviceSharedData(
            std::shared_ptr<std::mutex> master_mutex,
            std::shared_ptr<canopen::AsyncMaster> can_master,
            std::shared_ptr<ev::Executor> exec,
            uint8_t id)
        {
            this->master = can_master;
            this->master_mutex = master_mutex;
            this->exec = exec;
            this->id = id;
        }

        std::future<canopen::NmtState> reset_nmt_state_promise()
        {

            nmt_state_is_set.store(false);
            nmt_state_promise = std::promise<canopen::NmtState>();
            return nmt_state_promise.get_future();
        }

        void nmt_state_set(canopen::NmtState state)
        {
            if (!nmt_state_is_set.load())
            {
                //We do not care so much about missing a message, rather push them through.
                std::unique_lock<std::mutex> lk(nmt_mtex, std::defer_lock);
                if (lk.try_lock())
                {
                    nmt_state_is_set.store(true);
                    nmt_state_promise.set_value(state);
                }
            }
        }

        std::future<ros2_canopen::PDOData> reset_rpdo_promise()
        {
            std::scoped_lock<std::mutex> lk(pdo_mtex);
            rpdo_is_set.store(false);
            rpdo_promise = std::promise<ros2_canopen::PDOData>();
            return rpdo_promise.get_future();
        }

        void rpdo_set(PDOData data)
        {
            std::unique_lock<std::mutex> lk(pdo_mtex, std::defer_lock);
            if (lk.try_lock())
            {
                if (!rpdo_is_set.load())
                {
                    rpdo_is_set.store(true);
                    rpdo_promise.set_value(data);
                }
            }
        }
    };
    class BasicDeviceDriver : public canopen::FiberDriver
    {
        std::shared_ptr<ros2_canopen::BasicDeviceSharedData> shared_data;

    public:
        using FiberDriver::FiberDriver;
        BasicDeviceDriver(std::shared_ptr<ros2_canopen::BasicDeviceSharedData> shared_data)
            : FiberDriver(*(shared_data->exec), *(shared_data->master), shared_data->id)
        {
            this->shared_data = shared_data;
        }

    private:
        void
        OnState(canopen::NmtState state) noexcept override
        {
            canopen::NmtState st = state;
            //We assume 1F80 bit 2 is false. All slaves are put into Operational after boot-up.
            //Lelycore does not track NMT states in this mode except BOOTUP.
            if (st == canopen::NmtState::BOOTUP)
            {
                st = canopen::NmtState::START;
            }

            shared_data->nmt_state_set(st);
        }

        void
        OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
        {
            uint32_t data = (uint32_t)rpdo_mapped[idx][subidx];
            PDOData d(idx, subidx, data);
            shared_data->rpdo_set(d);
        }

        void
        OnEmcy(uint16_t eec, uint8_t er, uint8_t *msef) noexcept override
        {
        }
    };

    class BasicDriverNode : public rclcpp_lifecycle::LifecycleNode
    {
    private:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver;
        std::shared_ptr<ros2_canopen::BasicDeviceSharedData> shared_data;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> nmt_state_publisher;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ros2_canopen_interfaces::msg::PDO>> rpdo_publisher;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nmt_state_reset_service;
        std::atomic<bool> configured;
        std::future<void> nmt_state_publisher_future;
        std::future<void> rpdo_publisher_future;

        template <typename T>
        class WriteSdoCoTask : public ev::CoTask
        {
        private:
            uint16_t index;
            uint8_t subindex;
            T data;
            std::shared_ptr<canopen::FiberDriver> driver;
            std::promise<bool> p;

        public:
            void set_data(std::shared_ptr<canopen::FiberDriver> driver, uint16_t index, uint8_t subindex, T data)
            {
                this->index = index;
                this->subindex = subindex;
                this->data = data;
                this->driver = driver;
            }

            std::future<bool> get_future()
            {

                return p.get_future();
            }

            void operator()() noexcept
            {
                Wait(driver->AsyncWrite<T>(index, subindex, std::move(data)));
                p.set_value(true);
            }
        };

    public:
        explicit BasicDriverNode(
            const std::string &node_name,
            std::shared_ptr<BasicDeviceDriver> driver,
            std::shared_ptr<ros2_canopen::BasicDeviceSharedData> shared_data,
            bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(
                  node_name,
                  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {
            this->shared_data = shared_data;
            nmt_state_publisher = this->create_publisher<std_msgs::msg::String>(std::string(this->get_name()).append("/nmt_state").c_str(), 10);
            rpdo_publisher = this->create_publisher<ros2_canopen_interfaces::msg::PDO>(std::string(this->get_name()).append("/rpdo").c_str(), 10);
            nmt_state_reset_service = this->create_service<std_srvs::srv::Trigger>(
                std::string(this->get_name()).append("/nmt_reset_node").c_str(),
                std::bind(
                    &ros2_canopen::BasicDriverNode::nmt_state_reset_service_cb,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
            configured.store(false);
        }

        void on_nmt_state_change_cb()
        {
            while (configured.load())
            {
                auto f = shared_data->reset_nmt_state_promise();
                f.wait();
                on_nmt_state(f.get());
            }
        }
        void on_rdpo_cb()
        {
            while (configured.load())
            {
                auto f = shared_data->reset_rpdo_promise();
                f.wait();
                on_rpdo(f.get());
            }
        }

        void nmt_state_reset_service_cb(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            std::scoped_lock<std::mutex> lk(*(shared_data->master_mutex));
            shared_data->master->Command(canopen::NmtCommand::RESET_NODE, shared_data->id);
            response->success = true;
        }

        virtual void on_nmt_state(canopen::NmtState nmt_state)
        {
            auto message = std_msgs::msg::String();

            switch (nmt_state)
            {
            case canopen::NmtState::BOOTUP:
                message.data = "BOOTUP";
                break;
            case canopen::NmtState::PREOP:
                message.data = "PREOP";
                break;
            case canopen::NmtState::RESET_COMM:
                message.data = "RESET_COMM";
                break;
            case canopen::NmtState::RESET_NODE:
                message.data = "RESET_NODE";
                break;
            case canopen::NmtState::START:
                message.data = "START";
                break;
            case canopen::NmtState::STOP:
                message.data = "STOP";
                break;
            case canopen::NmtState::TOGGLE:
                message.data = "TOGGLE";
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown NMT State.");
                message.data = "ERROR";
                break;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Slave %hhu: Switched NMT state to %s",
                        this->shared_data->id,
                        message.data.c_str());
            nmt_state_publisher->publish(message);
        }

        virtual void on_rpdo(ros2_canopen::PDOData d)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Slave %hhu: Sent PDO index %hu, subindex %hhu, data %x",
                        this->shared_data->id,
                        d.index_,
                        d.subindex_,
                        d.data_);
            auto message = ros2_canopen_interfaces::msg::PDO();
            message.index = d.index_;
            message.subindex = d.subindex_;
            message.data = d.data_;
            rpdo_publisher->publish(message);
        }

        CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state)
        {
            configured.store(true);
            nmt_state_publisher_future = std::async(std::launch::async, std::bind(&ros2_canopen::BasicDriverNode::on_nmt_state_change_cb, this));
            rpdo_publisher_future = std::async(std::launch::async, std::bind(&ros2_canopen::BasicDriverNode::on_rdpo_cb, this));
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state)
        {
            nmt_state_publisher->on_activate();
            rpdo_publisher->on_activate();
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state)
        {
            nmt_state_publisher->on_deactivate();
            rpdo_publisher->on_deactivate();
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state)
        {
            configured.store(false);
            nmt_state_publisher_future.wait();
            rpdo_publisher_future.wait();
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state)
        {
            return CallbackReturn::SUCCESS;
        }
    };

    class BasicDevice : public CANopenDevice
    {
    public:
        void registerDriver(
            std::shared_ptr<ev::Executor> exec,
            std::shared_ptr<canopen::AsyncMaster> master,
            std::shared_ptr<std::mutex> master_mutex,
            uint8_t id)
        {
            /// Setup driver
            std::scoped_lock<std::mutex> lk(*master_mutex);
            std::string node_name = "BasicDevice";
            node_name.append(std::to_string(id));
            auto shared_data = std::make_shared<BasicDeviceSharedData>(master_mutex, master, exec, id);
            driver_ = std::make_shared<BasicDeviceDriver>(shared_data);
            driver_node_ = std::make_shared<BasicDriverNode>(node_name, driver_, shared_data);
        }

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node()
        {
            return driver_node_->get_node_base_interface();
        }

    private:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver_;
        std::shared_ptr<ros2_canopen::BasicDriverNode> driver_node_;
    };
}

#endif