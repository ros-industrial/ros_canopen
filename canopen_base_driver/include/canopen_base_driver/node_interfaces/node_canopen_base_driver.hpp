#ifndef NODE_CANOPEN_BASE_DRIVER
#define NODE_CANOPEN_BASE_DRIVER

#include "canopen_core/node_interfaces/node_canopen_driver.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_write.hpp"

namespace ros2_canopen
{
    namespace node_interfaces
    {
        template <class NODETYPE>
        class NodeCanopenBaseDriver : public NodeCanopenDriver<NODETYPE>
        {
            static_assert(
                std::is_base_of<rclcpp::Node, NODETYPE>::value ||
                    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
                "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

        protected:
            std::thread nmt_state_publisher_thread_;
            std::thread rpdo_publisher_thread_;
            std::mutex driver_mutex_;
            std::shared_ptr<ros2_canopen::LelyDriverBridge> lely_driver_;

            // nmt state callback
            std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb_;
            // rpdo callback
            std::function<void(COData, uint8_t)> rpdo_cb_;

            void nmt_listener();
            virtual void on_nmt(canopen::NmtState nmt_state);
            void rdpo_listener();
            virtual void on_rpdo(COData data);

        public:
            NodeCanopenBaseDriver(NODETYPE *node);

            virtual ~NodeCanopenBaseDriver()
            {
                if (nmt_state_publisher_thread_.joinable())
                {
                    nmt_state_publisher_thread_.join();
                }
                if (rpdo_publisher_thread_.joinable())
                {
                    rpdo_publisher_thread_.join();
                }
            }

            virtual void init(bool called_from_base);

            virtual void configure(bool called_from_base);

            virtual void activate(bool called_from_base);

            virtual void deactivate(bool called_from_base);

            virtual void cleanup(bool called_from_base);

            virtual void shutdown(bool called_from_base);

            virtual void add_to_master();

            virtual void remove_from_master();

            /**
             * @brief Register a callback for NMT state change
             *
             * @param nmt_state_cb
             */
            void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb)
            {
                nmt_state_cb_ = std::move(nmt_state_cb);
            }

            /**
             * @brief Register a callback for RPDO
             *
             * @param rpdo_cb
             */
            void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb)
            {
                rpdo_cb_ = std::move(rpdo_cb);
            }
        };
        typedef NodeCanopenBaseDriver<rclcpp::Node> NCBDNode;
        typedef NodeCanopenBaseDriver<rclcpp_lifecycle::LifecycleNode> NCBDLifecycleNode;
    }
}

#endif