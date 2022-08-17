#include "canopen_core/lifecycle_master_node.hpp"

namespace ros2_canopen
{
    void LifecycleMasterNode::init()
    {
        this->activated.store(false);
        //declare parameters
        this->declare_parameter("master_config", "");
        this->declare_parameter("master_bin", "");
        this->declare_parameter("can_interface_name", "");
        this->declare_parameter("node_id", 1);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleMasterNode::on_configure(const rclcpp_lifecycle::State &)
    {
        this->activated.store(false);
        // Fetch parameters
        this->get_parameter<std::string>("master_config", dcf_txt_);
        this->get_parameter<std::string>("master_bin", dcf_bin_); 
        this->get_parameter<std::string>("can_interface_name", can_interface_name_);
        this->get_parameter<uint8_t>("node_id", node_id_);

        //declare services
        sdo_read_service = this->create_service<canopen_interfaces::srv::COReadID>(
            std::string(this->get_name()).append("/sdo_read").c_str(),
            std::bind(
                &ros2_canopen::LifecycleMasterNode::on_sdo_read,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        sdo_write_service = this->create_service<canopen_interfaces::srv::COWriteID>(
            std::string(this->get_name()).append("/sdo_write").c_str(),
            std::bind(
                &ros2_canopen::LifecycleMasterNode::on_sdo_write,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
                
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleMasterNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        this->activated.store(true);
        io_guard_ = std::make_unique<lely::io::IoGuard>();
        ctx_ = std::make_unique<lely::io::Context>();
        poll_ = std::make_unique<lely::io::Poll>(*ctx_);
        loop_ = std::make_unique<lely::ev::Loop>(poll_->get_poll());

        exec_ = std::make_shared<lely::ev::Executor>(loop_->get_executor());
        timer_ = std::make_unique<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);
        ctrl_ = std::make_unique<io::CanController>(can_interface_name_.c_str());
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
            dcf_txt_, dcf_bin_, node_id_);
        master_->Reset();

        spinner_ = std::thread(
            [this]()
            {
                loop_->run();
            });

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleMasterNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        this->activated.store(false);
        exec_->post(
            [&](){
                ctx_->shutdown();
            }
        );
        RCLCPP_INFO(this->get_logger(), "Waiting for CANopen loop to shutdown.");
        spinner_.join();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleMasterNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        this->activated.store(false);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LifecycleMasterNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        this->activated.store(false);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void LifecycleMasterNode::init_driver(std::shared_ptr<ros2_canopen::LifecycleDriverInterface> node_instance, uint8_t node_id)
    {
        node_instance->init_from_master(exec_, master_, config_);
    }

    void LifecycleMasterNode::on_sdo_read(
        const std::shared_ptr<canopen_interfaces::srv::COReadID::Request> request,
        std::shared_ptr<canopen_interfaces::srv::COReadID::Response> response)
    {
        if (this->activated.load())
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
        else
        {
            RCLCPP_ERROR(this->get_logger(), "LifecycleMasterNode is not in active state. SDO read service is not available.");
            response->success = false;
        }
    }

    void LifecycleMasterNode::on_sdo_write(
        const std::shared_ptr<canopen_interfaces::srv::COWriteID::Request> request,
        std::shared_ptr<canopen_interfaces::srv::COWriteID::Response> response)
    {
        if (this->activated.load())
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
        else
        {
            RCLCPP_ERROR(this->get_logger(), "LifecycleMasterNode is not in active state. SDO write service is not available.");
            response->success = false;
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleMasterNode)