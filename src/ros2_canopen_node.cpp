/**
 * @file test_node.cpp
 * @author Christoph Hellmann Santos
 * @brief Testnode for integrating lelycore in a ros2 node
 * @version 0.0.1
 * @date 2022-01-19
 */
#include "ros2_canopen/ros2_canopen_node.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace lely;
using namespace ros2_canopen;

void ROSCANopen_Node::master_nmt(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterNmt::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterNmt::Response> response)
{
  if (active.load())
  {
    std::lock_guard<std::mutex> guard(*master_mutex);
    canopen::NmtCommand command = static_cast<canopen::NmtCommand>(request->nmtcommand);
    can_master->Command(command, request->nodeid);
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}
void ROSCANopen_Node::master_read_sdo8(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo8::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo8::Response> response)
{
  this->master_read<uint8_t, ros2_canopen_interfaces::srv::MasterReadSdo8::Request, ros2_canopen_interfaces::srv::MasterReadSdo8::Response>(request, response);
}

void ROSCANopen_Node::master_read_sdo16(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo16::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo16::Response> response)
{
  this->master_read<uint16_t, ros2_canopen_interfaces::srv::MasterReadSdo16::Request, ros2_canopen_interfaces::srv::MasterReadSdo16::Response>(request, response);
}

void ROSCANopen_Node::master_read_sdo32(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo32::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo32::Response> response)
{
  this->master_read<uint32_t, ros2_canopen_interfaces::srv::MasterReadSdo32::Request, ros2_canopen_interfaces::srv::MasterReadSdo32::Response>(request, response);
}

void ROSCANopen_Node::master_write_sdo8(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo8::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo8::Response> response)
{
  this->master_write<uint8_t, ros2_canopen_interfaces::srv::MasterWriteSdo8::Request, ros2_canopen_interfaces::srv::MasterWriteSdo8::Response>(request, response);
}

void ROSCANopen_Node::master_write_sdo16(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo16::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo16::Response> response)
{
  this->master_write<uint16_t, ros2_canopen_interfaces::srv::MasterWriteSdo16::Request, ros2_canopen_interfaces::srv::MasterWriteSdo16::Response>(request, response);
}

void ROSCANopen_Node::master_write_sdo32(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo32::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo32::Response> response)
{
  this->master_write<uint32_t, ros2_canopen_interfaces::srv::MasterWriteSdo32::Request, ros2_canopen_interfaces::srv::MasterWriteSdo32::Response>(request, response);
}

void ROSCANopen_Node::master_set_heartbeat(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterSetHeartbeat::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterSetHeartbeat::Response> response)
{
  if (active.load())
  {
    WriteSdoCoTask<uint16_t> write_task(*exec);
    write_task.set_data(can_master, request->nodeid, 0x1017, 0x0, request->heartbeat);
    auto f = write_task.get_future();
    {
      std::scoped_lock<std::mutex> lk(*master_mutex);
      exec->post(write_task);
    }
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
  else{
    RCLCPP_ERROR(this->get_logger(), "Couldn't set heartbeat because node not active");
    response->success = false;
  }
}

void ROSCANopen_Node::run()
{
  can_master->Reset();
  //Drivers need to be registered in same thread - they seem to start their event loop already.
  basicdevice = std::make_shared<ros2_canopen::BasicDevice>();
  basicdevice->registerDriver(exec, can_master, master_mutex,  2);
  while(!active.load()){
    std::this_thread::sleep_for(10ms);
  }
  while (active.load())
  {
    //get lock on canopen master executor
    std::scoped_lock<std::mutex> lk(*master_mutex);
    //do work for at max 5ms
    loop->run_one_for(5ms);
  }
}

void ROSCANopen_Node::read_yaml()
{
  YAML::Node node = YAML::LoadFile(yaml_path.c_str());
  for (
      YAML::const_iterator it_devices = node.begin();
      it_devices != node.end();
      it_devices++)
  {
    // Get toplevel node name
    std::string device_name = it_devices->first.as<std::string>();
    // Check that this is not master
    if (device_name.find("master") == std::string::npos)
    {
      // Device config
      YAML::Node config = it_devices->second;
      // Save in map
      int node_id = config["node_id"].as<int>();
      std::string driver = config["driver"].as<std::string>();
      drivers.insert({node_id, driver});
    }
  }
}

void ROSCANopen_Node::register_services()
{
  //Create service for master_nmt
  this->master_nmt_service = this->create_service<ros2_canopen_interfaces::srv::MasterNmt>(
      "master_nmt",
      std::bind(&ROSCANopen_Node::master_nmt,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  //Create service for read sdo
  this->master_read_sdo8_service = this->create_service<ros2_canopen_interfaces::srv::MasterReadSdo8>(
      "master_read8_sdo",
      std::bind(&ROSCANopen_Node::master_read_sdo8,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  this->master_read_sdo16_service = this->create_service<ros2_canopen_interfaces::srv::MasterReadSdo16>(
      "master_read16_sdo",
      std::bind(&ROSCANopen_Node::master_read_sdo16,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  this->master_read_sdo32_service = this->create_service<ros2_canopen_interfaces::srv::MasterReadSdo32>(
      "master_read32_sdo",
      std::bind(&ROSCANopen_Node::master_read_sdo32,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  //Create service for write sdo
  this->master_write_sdo8_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo8>(
      "master_write8_sdo",
      std::bind(&ROSCANopen_Node::master_write_sdo8,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  this->master_write_sdo16_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo16>(
      "master_write16_sdo",
      std::bind(&ROSCANopen_Node::master_write_sdo16,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  this->master_write_sdo32_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo32>(
      "master_write32_sdo",
      std::bind(&ROSCANopen_Node::master_write_sdo32,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  this->master_set_hearbeat_service = this->create_service<ros2_canopen_interfaces::srv::MasterSetHeartbeat>(
      "master_set_heartbeat",
      std::bind(&ROSCANopen_Node::master_set_heartbeat,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

CallbackReturn
ROSCANopen_Node::on_configure(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  this->configured.store(true);
  this->get_parameter("can_interface_name", can_interface_name);
  this->get_parameter("dcf_path", dcf_path);
  this->get_parameter("yaml_path", yaml_path);

  read_yaml();
  //Start loop
  io_guard = std::make_shared<io::IoGuard>();
  ctx = std::make_shared<io::Context>();
  poll = std::make_shared<io::Poll>(*ctx);
  loop = std::make_shared<ev::Loop>(poll->get_poll());
  exec = std::make_shared<ev::Executor>(loop->get_executor());
  can_timer = std::make_shared<io::Timer>(*poll, *exec, CLOCK_MONOTONIC);

  //@Todo: Probably read from parameter server
  ctrl = std::make_shared<io::CanController>(can_interface_name.c_str());
  chan = std::make_shared<io::CanChannel>(*poll, *exec);

  //Open CAN channel
  chan->open(*ctrl);

  //Create Master from DCF
  //@Todo: Probably read from parameter server
  can_master = std::make_shared<canopen::AsyncMaster>(*can_timer, *chan, dcf_path.c_str(), "", 1);
  run_f = std::async(std::launch::async, std::bind(&ROSCANopen_Node::run, this));
  std::this_thread::sleep_for(20ms);

  this->main_p.set_value();

  //@Todo: register drivers!
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_activate(const rclcpp_lifecycle::State &state)
{
  this->active.store(true);
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_deactivate(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  run_f.wait();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_cleanup(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  this->configured.store(false);
  canopen_loop_thread->join();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_shutdown(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  this->configured.store(true);
  canopen_loop_thread->join();
  return CallbackReturn::SUCCESS;
}

using namespace ros2_canopen;
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto canopen_node = std::make_shared<ROSCANopen_Node>("canopen_master");
  auto f = canopen_node->get_main_future();
  executor.add_node(canopen_node->get_node_base_interface());
  executor.spin_until_future_complete(f);
  // Add code to add driver nodes
  executor.add_node(canopen_node->get_node()->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}