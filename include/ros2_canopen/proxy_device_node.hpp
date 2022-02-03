#ifndef PROXY_DEVICE_NODE_HPP
#define PROXY_DEVICE_NODE_HPP

#include "ros2_canopen/basic_device_node.hpp"

namespace ros2_canopen
{
    class ProxyDeviceNode : public BasicDeviceNode
    {
    private:
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> nmt_state_publisher;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ros2_canopen_interfaces::msg::COData>> rpdo_publisher;
        rclcpp::Subscription<ros2_canopen_interfaces::msg::COData>::SharedPtr tpdo_subscriber;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nmt_state_reset_service;
        rclcpp::Service<ros2_canopen_interfaces::srv::CORead>::SharedPtr sdo_read_service;
        rclcpp::Service<ros2_canopen_interfaces::srv::COWrite>::SharedPtr sdo_write_service;

        std::mutex sdo_mtex;

        void on_nmt_state_reset(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        void on_sdo_read(
            const ros2_canopen_interfaces::srv::CORead::Request::SharedPtr request,
            ros2_canopen_interfaces::srv::CORead::Response::SharedPtr response);

        void on_sdo_write(
            const ros2_canopen_interfaces::srv::COWrite::Request::SharedPtr request,
            ros2_canopen_interfaces::srv::COWrite::Response::SharedPtr response);

        void on_tpdo(const ros2_canopen_interfaces::msg::COData::SharedPtr msg);

    public:
        explicit ProxyDeviceNode(
            const std::string &node_name,
            std::shared_ptr<BasicDeviceDriver> driver,
            bool intra_process_comms = false)
            : BasicDeviceNode(
                  node_name,
                  driver,
                  intra_process_comms)
        {
            nmt_state_publisher = this->create_publisher<std_msgs::msg::String>(std::string(this->get_name()).append("/nmt_state").c_str(), 10);
            tpdo_subscriber = this->create_subscription<ros2_canopen_interfaces::msg::COData>(
                std::string(this->get_name()).append("/tpdo").c_str(),
                10,
                std::bind(&ProxyDeviceNode::on_tpdo, this, std::placeholders::_1));

            rpdo_publisher = this->create_publisher<ros2_canopen_interfaces::msg::COData>(
                std::string(this->get_name()).append("/rpdo").c_str(), 10);

            nmt_state_reset_service = this->create_service<std_srvs::srv::Trigger>(
                std::string(this->get_name()).append("/nmt_reset_node").c_str(),
                std::bind(
                    &ros2_canopen::ProxyDeviceNode::on_nmt_state_reset,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            sdo_read_service = this->create_service<ros2_canopen_interfaces::srv::CORead>(
                std::string(this->get_name()).append("/sdo_read").c_str(),
                std::bind(
                    &ros2_canopen::ProxyDeviceNode::on_sdo_read,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            sdo_write_service = this->create_service<ros2_canopen_interfaces::srv::COWrite>(
                std::string(this->get_name()).append("/sdo_write").c_str(),
                std::bind(
                    &ros2_canopen::ProxyDeviceNode::on_sdo_write,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        }

    protected:
        virtual void on_nmt(canopen::NmtState nmt_state) override;
        virtual void on_rpdo(COData data) override;
        virtual CallbackReturn on_configure_app(const rclcpp_lifecycle::State &state) override;
        virtual CallbackReturn on_activate_app(const rclcpp_lifecycle::State &state) override;
        virtual CallbackReturn on_deactivate_app(const rclcpp_lifecycle::State &state) override;
        virtual CallbackReturn on_cleanup_app(const rclcpp_lifecycle::State &state) override;
        virtual CallbackReturn on_shuttdown_app(const rclcpp_lifecycle::State &state) override;
    };

    class ProxyDevice : public CANopenDevice
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
            std::string node_name = "basic_device_";
            node_name.append(std::to_string(id));
            driver_ = std::make_shared<BasicDeviceDriver>(*exec, *master, id, master_mutex);
            driver_node_ = std::make_shared<ProxyDeviceNode>(node_name, driver_);
        }

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node()
        {
            return driver_node_->get_node_base_interface();
        }

    private:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver_;
        std::shared_ptr<ros2_canopen::ProxyDeviceNode> driver_node_;
    };
};
#endif