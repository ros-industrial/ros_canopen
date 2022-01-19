/**
 * @file test_node.cpp
 * @author Christoph Hellmann Santos
 * @brief Testnode for integrating lelycore in a ros2 node
 * @version 0.0.1
 * @date 2022-01-19
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

//#include "ros2_canopen/cia402_driver.hpp"

#define ROS2_CONTROL_DEMO_HARDWARE_PUBLIC

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


using namespace lely;

class ROSCANopen_Node : public rclcpp_lifecycle::LifecycleNode {
  
  private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_ptr<io::IoGuard> io_guard;
  std::shared_ptr<io::Context> ctx;
  std::shared_ptr<io::Poll> poll;
  std::shared_ptr<ev::Loop> loop;
  std::shared_ptr<ev::Executor> exec;
  std::shared_ptr<io::CanController> ctrl;
  std::shared_ptr<io::CanChannel> chan;
  std::shared_ptr<io::Timer> can_timer;
  std::shared_ptr<canopen::AsyncMaster> can_master;
  //std::shared_ptr<CIA402_Driver> cia402_driver;
  std::shared_ptr<std::string> nmt_status;

  public:
  explicit  ROSCANopen_Node(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(
    node_name, 
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  { }

  void run_one(){
    loop->run_one();
  }

  CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) {
    io_guard = std::make_shared<io::IoGuard>();
    ctx = std::make_shared<io::Context>();
    poll = std::make_shared<io::Poll>(*ctx);
    loop = std::make_shared<ev::Loop>(poll->get_poll());
    exec = std::make_shared<ev::Executor>(loop->get_executor());
    can_timer = std::make_shared<io::Timer>(*poll, *exec, CLOCK_MONOTONIC);

    //@Todo: Probably read from parameter server
    ctrl = std::make_shared<io::CanController>("vcan0");
    chan = std::make_shared<io::CanChannel>(*poll, *exec);

    //Open CAN channel
    chan->open(*ctrl);

    //Create Master from DCF
    //@Todo: Probably read from parameter server
    can_master = std::make_shared<canopen::AsyncMaster>(*can_timer, *chan, "/home/christoph/master.dcf", "", 1);

    /// @Todo: Add Devices via Pluginlib?
    // For now simply add a driver
    //cia402_driver = std::make_shared<CIA402_Driver>(*exec, *can_master, 2, nmt_status);

    return CallbackReturn::SUCCESS;

  }

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) {
    // devices
    can_master->Reset();
    // run loop every 10ms
    timer_ = this->create_wall_timer(
      10ms, std::bind(&ROSCANopen_Node::run_one, this));
    

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) {
    ctx->shutdown();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) {
    ctx->shutdown();
    return CallbackReturn::SUCCESS;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto cia402_node = std::make_shared<ROSCANopen_Node>("test_node");
  executor.add_node(cia402_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}