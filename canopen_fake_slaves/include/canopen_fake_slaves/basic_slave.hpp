//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#ifndef BASIC_SLAVE_HPP
#define BASIC_SLAVE_HPP
#include <lely/coapp/slave.hpp>
#include <lely/ev/co_task.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <thread>

#include "canopen_fake_slaves/base_slave.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace lely;
using namespace std::chrono_literals;
namespace ros2_canopen
{
class SimpleSlave : public canopen::BasicSlave
{
public:
  using BasicSlave::BasicSlave;

protected:
  /**
   * @brief This function is called when a value is written to the local object dictionary by an SDO
   * or RPDO. Also copies the RPDO value to TPDO.
   * @param idx The index of the PDO.
   * @param subidx The subindex of the PDO.
   */
  void OnWrite(uint16_t idx, uint8_t subidx) noexcept override
  {
    // uint32_t val = (*this)[idx][subidx];
    //(*this)[0x4001][0] = val;
    // this->TpdoEvent(0);
  }
};

class BasicSlave : public BaseSlave
{
public:
  explicit BasicSlave(const std::string & node_name, bool intra_process_comms = false)
  : BaseSlave(node_name, intra_process_comms)
  {
  }

protected:
  class ActiveCheckTask : public ev::CoTask
  {
  public:
    ActiveCheckTask(io::Context * ctx, ev::Executor * exec, BasicSlave * slave) : CoTask(*exec)
    {
      slave_ = slave;
      exec_ = exec;
      ctx_ = ctx;
    }

  protected:
    BasicSlave * slave_;
    ev::Executor * exec_;
    io::Context * ctx_;
    virtual void operator()() noexcept
    {
      if (slave_->activated.load())
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

    auto sigset_ = lely::io::SignalSet(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset_.insert(SIGHUP);
    sigset_.insert(SIGINT);
    sigset_.insert(SIGTERM);

    sigset_.submit_wait(
      [&](int /*signo*/)
      {
        // If the signal is raised again, terminate immediately.
        sigset_.clear();

        // Perform a clean shutdown.
        ctx.shutdown();
      });

    SimpleSlave slave(timer, chan, slave_config_.c_str(), "", node_id_);
    slave.Reset();
    ActiveCheckTask checktask(&ctx, &exec, this);

    // timer.submit_wait()
    RCLCPP_INFO(this->get_logger(), "Created slave for node_id %i.", node_id_);
    loop.run();
    ctx.shutdown();
    RCLCPP_INFO(this->get_logger(), "Stopped CANopen Event Loop.");
    rclcpp::shutdown();
  }
};
}  // namespace ros2_canopen

#endif
