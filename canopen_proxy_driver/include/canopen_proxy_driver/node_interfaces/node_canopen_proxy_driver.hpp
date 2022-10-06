#ifndef NODE_CANOPEN_PROXY_DRIVER
#define NODE_CANOPEN_PROXY_DRIVER

#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
namespace ros2_canopen
{
    namespace node_interfaces
    {

        template <class NODETYPE>
        class NodeCanopenProxyDriver : public NodeCanopenBaseDriver<NODETYPE>
        {
            static_assert(
                std::is_base_of<rclcpp::Node, NODETYPE>::value ||
                    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
                "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

        protected:
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nmt_state_publisher;
            rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr rpdo_publisher;
            rclcpp::Subscription<canopen_interfaces::msg::COData>::SharedPtr tpdo_subscriber;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nmt_state_reset_service;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nmt_state_start_service;
            rclcpp::Service<canopen_interfaces::srv::CORead>::SharedPtr sdo_read_service;
            rclcpp::Service<canopen_interfaces::srv::COWrite>::SharedPtr sdo_write_service;

            std::mutex sdo_mtex;

            virtual void on_nmt(canopen::NmtState nmt_state) override;
            virtual void on_rpdo(COData data) override;
            virtual void on_tpdo(const canopen_interfaces::msg::COData::SharedPtr msg);

            void on_nmt_state_reset(
                const std_srvs::srv::Trigger::Request::SharedPtr request,
                std_srvs::srv::Trigger::Response::SharedPtr response);

            void on_nmt_state_start(
                const std_srvs::srv::Trigger::Request::SharedPtr request,
                std_srvs::srv::Trigger::Response::SharedPtr response);

            void on_sdo_read(
                const canopen_interfaces::srv::CORead::Request::SharedPtr request,
                canopen_interfaces::srv::CORead::Response::SharedPtr response);

            void on_sdo_write(
                const canopen_interfaces::srv::COWrite::Request::SharedPtr request,
                canopen_interfaces::srv::COWrite::Response::SharedPtr response);

        public:
            NodeCanopenProxyDriver(NODETYPE *node);

            virtual void init(bool called_from_base) override;

            virtual bool reset_node_nmt_command();

            virtual bool start_node_nmt_command();

            virtual bool tpdo_transmit(COData& data);

            virtual bool sdo_write(COData& data);

            virtual bool sdo_read(COData& data);
        };
    }
}

#endif