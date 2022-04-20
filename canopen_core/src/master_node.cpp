#include "canopen_core/master_node.hpp"

namespace ros2_canopen
{
    void MasterNode::init(std::string dcf_txt, std::string dcf_bin, std::string can_interface_name, uint8_t nodeid, 
                            std::shared_ptr<ConfigurationManager> config)
    {
        MasterInterface::init(dcf_txt, dcf_bin, can_interface_name, nodeid, config);

        io_guard_ = std::make_unique<lely::io::IoGuard>();
        ctx_ = std::make_unique<lely::io::Context>();
        poll_ = std::make_unique<lely::io::Poll>(*ctx_);
        loop_ = std::make_unique<lely::ev::Loop>(poll_->get_poll());

        exec_ = std::make_shared<lely::ev::Executor>(loop_->get_executor());
        timer_ = std::make_unique<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);
        ctrl_ = std::make_unique<io::CanController>(can_interface_name.c_str());
        chan_ = std::make_unique<io::CanChannel>(*poll_, *exec_);
        chan_->open(*ctrl_);

        sigset_ = std::make_unique<io::SignalSet>(*poll_, *exec_);
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

        master_ = std::make_shared<LelyMasterBridge>(
            *exec_, *timer_, *chan_,
            dcf_txt, dcf_bin, nodeid);
        master_->Reset();

        spinner_ = std::thread(
            [this]()
            {
                loop_->run();
            });

        /// @todo add services
        sdo_read_service = this->create_service<canopen_interfaces::srv::COReadID>(
            std::string(this->get_name()).append("/sdo_read").c_str(),
            std::bind(
                &ros2_canopen::MasterNode::on_sdo_read,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        sdo_write_service = this->create_service<canopen_interfaces::srv::COWriteID>(
            std::string(this->get_name()).append("/sdo_write").c_str(),
            std::bind(
                &ros2_canopen::MasterNode::on_sdo_write,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    }
    void MasterNode::add_driver(std::shared_ptr<ros2_canopen::DriverInterface> node_instance, uint8_t node_id)
    {
        std::shared_ptr<std::promise<void>> prom = std::make_shared<std::promise<void>>();
        auto f = prom->get_future();
        exec_->post([this, node_id, node_instance, prom]()
                    {
                            node_instance->init(*this->exec_, *(this->master_), node_id, config_);
                            prom->set_value(); });
        f.wait();
    }

    void MasterNode::remove_driver(std::shared_ptr<ros2_canopen::DriverInterface> node_instance, uint8_t node_id)
    {
        std::shared_ptr<std::promise<void>> prom = std::make_shared<std::promise<void>>();
        auto f = prom->get_future();
        exec_->post([this, node_id, node_instance, prom]()
                    { 
                            node_instance->remove(*this->exec_, *(this->master_), node_id); 
                            prom->set_value(); });
        f.wait();
    }

    void MasterNode::on_sdo_read(
        const std::shared_ptr<canopen_interfaces::srv::COReadID::Request> request,
        std::shared_ptr<canopen_interfaces::srv::COReadID::Response> response)
    {
        ros2_canopen::CODataTypes datatype = static_cast<ros2_canopen::CODataTypes>(request->type);
        ros2_canopen::COData data = {request->index, request->subindex, 0U, datatype};
        std::future<COData> f = this->master_->async_read_sdo(request->nodeid, data);
        f.wait();
        try
        {
            response->data = f.get().data_;
            response->success = true;
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            response->success = false;
        }
    }

    void MasterNode::on_sdo_write(
        const std::shared_ptr<canopen_interfaces::srv::COWriteID::Request> request,
        std::shared_ptr<canopen_interfaces::srv::COWriteID::Response> response)
    {
        ros2_canopen::CODataTypes datatype = static_cast<ros2_canopen::CODataTypes>(request->type);
        ros2_canopen::COData data = {request->index, request->subindex, request->data, datatype};
        std::future<bool> f = this->master_->async_write_sdo(request->nodeid, data);
        f.wait();
        try
        {
            response->success = f.get();
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            response->success = false;
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::MasterNode)