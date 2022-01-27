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
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Couldn't set heartbeat because node not active");
    response->success = false;
  }
}

void ROSCANopen_Node::run()
{
  can_master->Reset();
  register_drivers();

  //Drivers are set and nodes can b added to ros executor, wait for them to be added.
  this->post_registration.set_value();
    // Signal to ROS Node that drivers were success fully registered.
  this->registration_done.set_value();

  this->post_registration_done.wait();

  while (configured)
  {
    //Get future from current active promise
    auto active_f = this->active_p.get_future();
    //Wait for future to become ready
    active_f.wait();
    //while node is active do the work.
    while (active.load())
    {
      //get lock on canopen master executor
      std::scoped_lock<std::mutex> lk(*master_mutex);
      //do work for at max 5ms
      loop->run_one_for(5ms);
    }
  }
  this->pre_deregistration.set_value();
  this->pre_deregistration_done.wait();
  //Safe to delete devices.
  this->deregister_drivers();
  this->drivers = std::map<int, std::string>();
  this->ctx->shutdown();
}

void ROSCANopen_Node::register_drivers()
{
  for (auto it = this->drivers.begin(); it != this->drivers.end(); ++it)
  {
    uint8_t id = (uint8_t)it->first;
    std::string name = it->second;
    if (name.compare("BasicDevice") == 0)
    {
      auto dev = std::make_shared<ros2_canopen::BasicDevice>();
      this->devices->insert({id, dev});
      dev->registerDriver(exec, can_master, master_mutex, id);
    }
  }
}

void ROSCANopen_Node::deregister_drivers()
{
  for (auto it = this->devices->begin(); it != this->devices->end(); ++it)
  {
    it->second->get_node()->~NodeBaseInterface();
    this->devices->erase(it);
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
      std::string(this->get_name()).append("/master_nmt").c_str(),
      std::bind(&ROSCANopen_Node::master_nmt,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  //Create service for read sdo
  this->master_read_sdo8_service = this->create_service<ros2_canopen_interfaces::srv::MasterReadSdo8>(
      std::string(this->get_name()).append("/master_read8_sdo").c_str(),
      std::bind(&ROSCANopen_Node::master_read_sdo8,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  this->master_read_sdo16_service = this->create_service<ros2_canopen_interfaces::srv::MasterReadSdo16>(
      std::string(this->get_name()).append("/master_read16_sdo").c_str(),
      std::bind(&ROSCANopen_Node::master_read_sdo16,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  this->master_read_sdo32_service = this->create_service<ros2_canopen_interfaces::srv::MasterReadSdo32>(
      std::string(this->get_name()).append("/master_read32_sdo").c_str(),
      std::bind(&ROSCANopen_Node::master_read_sdo32,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  //Create service for write sdo
  this->master_write_sdo8_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo8>( 
      std::string(this->get_name()).append("/master_write8_sdo").c_str(),
      std::bind(&ROSCANopen_Node::master_write_sdo8,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  this->master_write_sdo16_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo16>(
      std::string(this->get_name()).append("/master_write16_sdo").c_str(),
      std::bind(&ROSCANopen_Node::master_write_sdo16,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  this->master_write_sdo32_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo32>(
      std::string(this->get_name()).append("/master_write32_sdo").c_str(),
      std::bind(&ROSCANopen_Node::master_write_sdo32,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  this->master_set_hearbeat_service = this->create_service<ros2_canopen_interfaces::srv::MasterSetHeartbeat>(
      std::string(this->get_name()).append("/master_set_heartbeat").c_str(),
      std::bind(&ROSCANopen_Node::master_set_heartbeat,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

CallbackReturn
ROSCANopen_Node::on_configure(const rclcpp_lifecycle::State &state)
{
  //Wait for master Thread to terminate.
  if(master_thread_running.valid())
    master_thread_running.wait();
  
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
  master_thread_running = std::async(std::launch::async, std::bind(&ROSCANopen_Node::run, this));

  //Create new promise
  active_p = std::promise<void>();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_activate(const rclcpp_lifecycle::State &state)
{
  //Wait for Drivers to be registered - in case it is called to fast.
  registration_done.get_future().wait();
  //Set active to true
  this->active.store(true);
  //Signal to master thread that we are active, to start loop.
  active_p.set_value();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_deactivate(const rclcpp_lifecycle::State &state)
{
  //Recreate promise so that master can wait for new future.
  active_p = std::promise<void>();
  //Stop master loop.
  this->active.store(false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_cleanup(const rclcpp_lifecycle::State &state)
{
  this->configured.store(false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_shutdown(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  this->configured.store(true);
  return CallbackReturn::SUCCESS;
}

using namespace ros2_canopen;
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto canopen_node = std::make_shared<ROSCANopen_Node>("canopen_master");
  executor.add_node(canopen_node->get_node_base_interface());
  auto devices = canopen_node->get_driver_nodes();

  while (true)
  {
    //////////////////////////////
    // Unconfigured State
    //////////////////////////////

    //Set up synchronisation
    std::promise<void> post_reg_p;
    std::promise<void> pre_dereg_done_p;

    //Get future that signals drivers were registered
    auto post_reg_f = canopen_node->get_post_registration_future(post_reg_p.get_future());
    
    //Spin until drivers were registered.
    auto ret = executor.spin_until_future_complete(post_reg_f);

    //Break when interrupted
    if (ret == rclcpp::FutureReturnCode::INTERRUPTED)
      break;
    
    //Add driver nodes to executor
    if (devices->size() > 0)
    {
      for (auto it = devices->begin(); it != devices->end(); it++)
      {
        executor.add_node(it->second->get_node());
      }
    }

    //Now create handshake for deconfiguring
    auto pre_dereg_f = canopen_node->get_pre_deregistration_future(pre_dereg_done_p.get_future());
    //Signal that driver nodes were added to executor and it is safe to go on.
    post_reg_p.set_value();

    //////////////////////////////
    // Configured State
    //////////////////////////////

    //Wait for deconfigure to remove driver nodes.
    ret = executor.spin_until_future_complete(pre_dereg_f);
    if (ret == rclcpp::FutureReturnCode::INTERRUPTED)
      break;


    //If there are drivers present, remove from executor.
    if (devices->size() > 0)
    {
      for (auto it = devices->begin(); it != devices->end(); it++)
      {
        executor.remove_node(it->second->get_node());
      }
    }
    //Signal to master threat, that nodes were removed and its safe to remove devices.
    pre_dereg_done_p.set_value();
  }

  rclcpp::shutdown();
  return 0;
}