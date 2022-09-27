#ifndef NODE_CANOPEN_MASTER_HPP_
#define NODE_CANOPEN_MASTER_HPP_

#include "canopen_core/node_interfaces/node_canopen_master_interface.hpp"
#include "canopen_core/master_error.hpp"
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <lely/coapp/master.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/ev/exec.hpp>
#include <lely/ev/loop.hpp>

namespace ros2_canopen
{
    namespace node_interfaces
    {
        template <class NODETYPE>
        class NodeCanopenMaster : public NodeCanopenMasterInterface
        {
            static_assert(
                std::is_base_of<rclcpp::Node, NODETYPE>::value ||
                    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
                "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

        protected:
            NODETYPE *node_;

            std::atomic<bool> initialised_;
            std::atomic<bool> configured_;
            std::atomic<bool> activated_;
            std::atomic<bool> master_set_;

            std::shared_ptr<lely::canopen::AsyncMaster> master_;
            std::shared_ptr<lely::ev::Executor> exec_;

            std::unique_ptr<lely::io::IoGuard> io_guard_;
            std::unique_ptr<lely::io::Context> ctx_;
            std::unique_ptr<lely::io::Poll> poll_;
            std::unique_ptr<lely::ev::Loop> loop_;
            std::unique_ptr<lely::io::Timer> timer_;
            std::unique_ptr<lely::io::CanController> ctrl_;
            std::unique_ptr<lely::io::CanChannel> chan_;
            std::unique_ptr<lely::io::SignalSet> sigset_;

            rclcpp::CallbackGroup::SharedPtr client_cbg_;
            rclcpp::CallbackGroup::SharedPtr timer_cbg_;

            YAML::Node config_;
            uint8_t node_id_;
            std::chrono::milliseconds non_transmit_timeout_;
            std::string container_name_;
            std::string master_dcf_;
            std::string master_bin_;
            std::string can_interface_;

            std::thread spinner_;

        public:
            NodeCanopenMaster(NODETYPE *node)
                : initialised_(false),
                  configured_(false),
                  activated_(false),
                  master_set_(false)
            {
                node_ = node;
            }

            virtual ~NodeCanopenMaster()
            {
                if(this->spinner_.joinable())
                {
                    RCLCPP_INFO(node_->get_logger(), "Joining Spinner Thread.");
                    this->spinner_.join();
                }
            }

            /**
             * @brief Initialise Master
             *
             */
            void init() override
            {
                RCLCPP_DEBUG(node_->get_logger(), "init_start");
                if (configured_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterAlreadyConfigured, MasterErrorCategory(), "Init");
                }
                if (activated_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterAlreadyActivated, MasterErrorCategory(), "Init");
                }
                client_cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                timer_cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                node_->declare_parameter("container_name", "");
                node_->declare_parameter("master_dcf", "");
                node_->declare_parameter("master_bin", "");
                node_->declare_parameter("can_interface", "vcan0");
                node_->declare_parameter("node_id", 0);
                node_->declare_parameter("non_transmit_timeout", 100);
                node_->declare_parameter("config", "");
                this->init(true);
                this->initialised_.store(true);
                RCLCPP_DEBUG(node_->get_logger(), "init_end");
            }
            virtual void init(bool called_from_base)
            {

            }

            /**
             * @brief Configure the driver
             *
             * This function should contain the configuration related things,
             * such as reading parameter data or configuration data from files.
             *
             */
            void configure() override
            {
                if (!initialised_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotInitialised, MasterErrorCategory(), "Configure");
                }
                if (configured_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterAlreadyConfigured, MasterErrorCategory(), "Configure");
                }
                if (activated_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterAlreadyActivated, MasterErrorCategory(), "Configure");
                }

                int non_transmit_timeout;
                std::string config;

                node_->get_parameter("container_name", container_name_);
                node_->get_parameter("master_dcf", master_dcf_);
                node_->get_parameter("master_bin", master_bin_);
                node_->get_parameter("can_interface", can_interface_);
                node_->get_parameter("node_id", node_id_);
                node_->get_parameter("non_transmit_timeout", non_transmit_timeout);
                node_->get_parameter("config", config);

                this->config_ = YAML::Load(config);
                this->non_transmit_timeout_ = std::chrono::milliseconds(non_transmit_timeout);

                this->configure(true);
                this->configured_.store(true);
            }

            virtual void configure(bool called_from_base)
            {
                
            }

            /**
             * @brief Activate the driver
             *
             * This function should activate the driver, consequently it needs to start all timers and threads necessary
             * for the operation of the driver.
             *
             */
            void activate() override
            {
                if (!initialised_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotInitialised, MasterErrorCategory(), "Activate");
                }
                if (!configured_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotConfigured, MasterErrorCategory(), "Activate");
                }
                if (activated_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterAlreadyActivated, MasterErrorCategory(), "Activate");
                }

                io_guard_ = std::make_unique<lely::io::IoGuard>();
                ctx_ = std::make_unique<lely::io::Context>();
                poll_ = std::make_unique<lely::io::Poll>(*ctx_);
                loop_ = std::make_unique<lely::ev::Loop>(poll_->get_poll());

                exec_ = std::make_shared<lely::ev::Executor>(loop_->get_executor());
                timer_ = std::make_unique<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);
                ctrl_ = std::make_unique<lely::io::CanController>(can_interface_.c_str());
                chan_ = std::make_unique<lely::io::CanChannel>(*poll_, *exec_);
                chan_->open(*ctrl_);

                sigset_ = std::make_unique<lely::io::SignalSet>(*poll_, *exec_);
                // Watch for Ctrl+C or process termination.
                sigset_->insert(SIGHUP);
                sigset_->insert(SIGINT);
                sigset_->insert(SIGTERM);

                sigset_->submit_wait(
                    [&](int /*signo*/)
                    {
                        // If the signal is raised again, terminate immediately.
                        sigset_->clear();

                        // Perform a clean shutdown.
                        ctx_->shutdown();
                    });

                this->activate(true);
                if(!master_)
                {
                    throw std::system_error(MasterErrorCode::MasterNotMasterSet, MasterErrorCategory(), "activate");
                }
                this->master_set_.store(true);
                this->spinner_ = std::thread(
                    [this]()
                    {
                        while(rclcpp::ok())
                        {
                            loop_->run_for(std::chrono::milliseconds(100));
                        }

                    });
                this->activated_.store(true);
            }
            /**
             * @brief Activate hook for derived classes
             *
             * This function should create a Master using exec_, timer_, master_dcf_, master_bin_
             * and node_id_ members and store it in master_. It should also create a thread and run
             * the master's event loop.
             *
             * @param called_from_base
             */
            virtual void activate(bool called_from_base)
            {
                
            }

            /**
             * @brief Deactivate the driver
             *
             * This function should deactivate the driver, consequently it needs to stop all timers and threads that
             * are related to the operation of the diver.
             *
             */
            void deactivate() override
            {
                if (!initialised_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotInitialised, MasterErrorCategory(), "Deactivate");
                }
                if (!configured_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotConfigured, MasterErrorCategory(), "Deactivate");
                }
                if (!activated_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotActivated, MasterErrorCategory(), "Deactivate");
                }

                exec_->post(
                    [&]()
                    {
                        ctx_->shutdown();
                    });

                this->deactivate(true);
                this->activated_.store(false);
            }

            /**
             * @brief Deactivate hook for derived classes
             *
             * This function should wait to join the thread created in the activate
             * function.
             *
             * @param called_from_base
             */
            virtual void deactivate(bool called_from_base)
            {
                
            }

            /**
             * @brief Cleanup the driver
             *
             * This function needs to clean the internal state of the driver. This means
             * all data should be deleted.
             *
             */
            void cleanup() override
            {
                if (!initialised_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotInitialised, MasterErrorCategory(), "Cleanup");
                }
                if (!configured_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotConfigured, MasterErrorCategory(), "Cleanup");
                }
                if (activated_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterAlreadyActivated, MasterErrorCategory(), "Cleanup");
                }
                this->cleanup(true);
                this->configured_.store(false);
            }
            virtual void cleanup(bool called_from_base)
            {
                
            }

            /**
             * @brief Shutdown the driver
             *
             * This function should shutdown the driver.
             *
             */
            void shutdown() override
            {
                this->master_set_.store(false);
                this->initialised_.store(false);
                this->configured_.store(false);
                this->activated_.store(false);
                shutdown(true);
            }
            virtual void shutdown(bool called_from_base)
            {
                
            }

            /**
             * @brief Get the master object
             *
             * @return std::shared_ptr<lely::canopen::AsyncMaster>
             */
            virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master()
            {
                if (!master_set_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotMasterSet, MasterErrorCategory(), "get_master");
                }
                return master_;
            }
            /**
             * @brief Get the executor object
             *
             * @return std::shared_ptr<lely::canopen::Executor>
             */
            virtual std::shared_ptr<lely::ev::Executor> get_executor()
            {
                if (!master_set_.load())
                {
                    throw std::system_error(MasterErrorCode::MasterNotMasterSet, MasterErrorCategory(), "get_executor");
                }
                return exec_;
            }
        };
    }
}

#endif