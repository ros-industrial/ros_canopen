#ifndef NODE_CANOPEN_BASIC_MASTER_HPP_
#define NODE_CANOPEN_BASIC_MASTER_HPP_

#include "canopen_core/node_interfaces/node_canopen_master.hpp"
#include "canopen_master_driver/lely_master_bridge.hpp"
#include "canopen_interfaces/srv/co_read_id.hpp"
#include "canopen_interfaces/srv/co_write_id.hpp"

namespace ros2_canopen
{
    namespace node_interfaces
    {
        template <class NODETYPE>
        class NodeCanopenBasicMaster : public ros2_canopen::node_interfaces::NodeCanopenMaster<NODETYPE>
        {
            static_assert(
                std::is_base_of<rclcpp::Node, NODETYPE>::value ||
                    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
                "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

        protected:
            std::shared_ptr<LelyMasterBridge> master_bridge_;
            rclcpp::Service<canopen_interfaces::srv::COReadID>::SharedPtr sdo_read_service;
            rclcpp::Service<canopen_interfaces::srv::COWriteID>::SharedPtr sdo_write_service;

        public:
            NodeCanopenBasicMaster(NODETYPE *node) : ros2_canopen::node_interfaces::NodeCanopenMaster<NODETYPE>(node)
            {
            }
            
            virtual void init(bool called_from_base) override
            {
            }

            virtual void configure(bool called_from_base) override
            {
            }
            virtual void activate(bool called_from_base) override
            {
                master_bridge_ = std::make_shared<LelyMasterBridge>(*(this->exec_), *(this->timer_), *(this->chan_), this->master_dcf_, this->master_bin_, this->node_id_);
                this->master_ = master_bridge_;
            }
            virtual void deactivate(bool called_from_base) override
            {
            }
            virtual void cleanup(bool called_from_base) override
            {
            }
            virtual void shutdown(bool called_from_base) override
            {
            }
            /**
             * @brief Read Service Data Object
             *
             * This Service is only available when the node is in active lifecycle state.
             * It will return with success false in any other lifecycle state and log an
             * RCLCPP_ERROR.
             *
             * @param request
             * @param response
             */
            void on_sdo_read(
                const std::shared_ptr<canopen_interfaces::srv::COReadID::Request> request,
                std::shared_ptr<canopen_interfaces::srv::COReadID::Response> response);

            /**
             * @brief Write Service Data Object
             *
             * This service is only available when the node is in active lifecycle state.
             * It will return with success false in any other lifecycle state and log an
             * RCLCPP_ERROR.
             *
             * @param request
             * @param response
             */
            void on_sdo_write(
                const std::shared_ptr<canopen_interfaces::srv::COWriteID::Request> request,
                std::shared_ptr<canopen_interfaces::srv::COWriteID::Response> response);
        };
    }
}

#endif