#ifndef NODE_CANOPEN_MASTER_INTERFACE_HPP_
#define NODE_CANOPEN_MASTER_INTERFACE_HPP_

#include <lely/coapp/master.hpp>
#include <lely/ev/exec.hpp>
#include "canopen_core/node_interfaces/node_canopen_driver_interface.hpp"

namespace ros2_canopen
{
    namespace node_interfaces
    {
        class NodeCanopenMasterInterface
        {
        public:
            /**
             * @brief Initialise Master
             *
             */
            virtual void init() = 0;

            /**
             * @brief Configure the driver
             *
             * This function should contain the configuration related things,
             * such as reading parameter data or configuration data from files.
             *
             */
            virtual void configure() = 0;

            /**
             * @brief Activate the driver
             *
             * This function should activate the driver, consequently it needs to start all timers and threads necessary
             * for the operation of the driver.
             *
             */
            virtual void activate() = 0;

            /**
             * @brief Deactivate the driver
             *
             * This function should deactivate the driver, consequently it needs to stop all timers and threads that
             * are related to the operation of the diver.
             *
             */
            virtual void deactivate() = 0;

            /**
             * @brief Cleanup the driver
             *
             * This function needs to clean the internal state of the driver. This means
             * all data should be deleted.
             *
             */
            virtual void cleanup() = 0;

            /**
             * @brief Shutdown the driver
             *
             * This function should shutdown the driver.
             *
             */
            virtual void shutdown() = 0;
            /**
             * @brief Get the master object
             * 
             * @return std::shared_ptr<lely::canopen::AsyncMaster> 
             */
            virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() = 0;
            /**
             * @brief Get the executor object
             * 
             * @return std::shared_ptr<lely::canopen::Executor> 
             */
            virtual std::shared_ptr<lely::ev::Executor> get_executor() = 0;
        };
    }
}
#endif
