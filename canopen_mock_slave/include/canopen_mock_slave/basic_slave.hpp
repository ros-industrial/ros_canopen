#ifndef BASIC_SLAVE_HPP
#define BASIC_SLAVE_HPP
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/ev/co_task.hpp>


#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "canopen_mock_slave/base_slave.hpp"

using namespace lely;
using namespace std::chrono_literals;
namespace ros2_canopen
{
    class BasicSlave : public BaseSlave
    {
    public:
        explicit BasicSlave(const std::string &node_name, bool intra_process_comms = false) : BaseSlave(node_name, intra_process_comms)
        {
        }

    protected:
        class ActiveCheckTask : public ev::CoTask
        {
            public:
            ActiveCheckTask(io::Context* ctx, ev::Executor* exec, BasicSlave* slave) : CoTask(*exec)
            {
                slave_ = slave;
                exec_ = exec;
                ctx_ = ctx;

            }

            protected:
            BasicSlave* slave_;
            ev::Executor *exec_;
            io::Context *ctx_;
            virtual void operator() () noexcept
            {
                    if(slave_->activated.load())
                    {
                        
                    }
                    ctx_->shutdown();
            }
        };

        void run() override
        {
            io::IoGuard io_guard;
            io::Context ctx;
            io::Poll poll(ctx);
            ev::Loop loop(poll.get_poll());
            auto exec = loop.get_executor();
            io::Timer timer(poll, exec, CLOCK_MONOTONIC);
            io::CanController ctrl(can_interface_name_.c_str());
            io::CanChannel chan(poll, exec);
            chan.open(ctrl);
            canopen::BasicSlave slave(timer, chan, slave_config_.c_str(), "", node_id_);
            slave.Reset();
            ActiveCheckTask checktask(&ctx, &exec, this);
            //exec.post(checktask);
            RCLCPP_INFO(this->get_logger(), "Created slave for node_id %i.", node_id_);
            loop.run();
            ctx.shutdown();
            RCLCPP_INFO(this->get_logger(), "Stopped CANopen Event Loop.");
        }
    };
}

#endif