// #include <ros/package.h>

#include <canopen_chain_node/ros_chain.hpp>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int64.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>

using namespace can;

namespace canopen
{

// template<typename Tpub, int dt>
// static PublishFuncType create(ros::NodeHandle &nh,  const std::string &name, ObjectStorageSharedPtr storage, const std::string &key, const bool force){
//     using data_type = typename ObjectStorage::DataType<dt>::type;
//     using entry_type = ObjectStorage::Entry<data_type>;
//
//     entry_type entry = storage->entry<data_type>(key);
//     if(!entry.valid()) return 0;
//
//     const ros::Publisher pub = nh.advertise<Tpub>(name, 1);
//
//     typedef const data_type(entry_type::*getter_type)(void);
//     const getter_type getter = force ? static_cast<getter_type>(&entry_type::get) : static_cast<getter_type>(&entry_type::get_cached);
//
//     return [force, pub, entry, getter] () mutable {
//         Tpub msg;
//         msg.data = (const typename Tpub::_data_type &) (entry.*getter)();
//         pub.publish(msg);
//     };
// }

// PublishFuncType createPublishFunc(ros::NodeHandle &nh,  const std::string &name, canopen::NodeSharedPtr node, const std::string &key, bool force){
//     ObjectStorageSharedPtr s = node->getStorage();
//
//     switch(ObjectDict::DataTypes(s->dict_->get(key)->data_type)){
//         case ObjectDict::DEFTYPE_INTEGER8:       return create< std_msgs::Int8,    ObjectDict::DEFTYPE_INTEGER8       >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_INTEGER16:      return create< std_msgs::Int16,   ObjectDict::DEFTYPE_INTEGER16      >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_INTEGER32:      return create< std_msgs::Int32,   ObjectDict::DEFTYPE_INTEGER32      >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_INTEGER64:      return create< std_msgs::Int64,   ObjectDict::DEFTYPE_INTEGER64      >(nh, name, s, key, force);
//
//         case ObjectDict::DEFTYPE_UNSIGNED8:      return create< std_msgs::UInt8,   ObjectDict::DEFTYPE_UNSIGNED8      >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_UNSIGNED16:     return create< std_msgs::UInt16,  ObjectDict::DEFTYPE_UNSIGNED16     >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_UNSIGNED32:     return create< std_msgs::UInt32,  ObjectDict::DEFTYPE_UNSIGNED32     >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_UNSIGNED64:     return create< std_msgs::UInt64,  ObjectDict::DEFTYPE_UNSIGNED64     >(nh, name, s, key, force);
//
//         case ObjectDict::DEFTYPE_REAL32:         return create< std_msgs::Float32, ObjectDict::DEFTYPE_REAL32         >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_REAL64:         return create< std_msgs::Float64, ObjectDict::DEFTYPE_REAL64         >(nh, name, s, key, force);
//
//         case ObjectDict::DEFTYPE_VISIBLE_STRING: return create< std_msgs::String,  ObjectDict::DEFTYPE_VISIBLE_STRING >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_OCTET_STRING:   return create< std_msgs::String,  ObjectDict::DEFTYPE_DOMAIN         >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_UNICODE_STRING: return create< std_msgs::String,  ObjectDict::DEFTYPE_UNICODE_STRING >(nh, name, s, key, force);
//         case ObjectDict::DEFTYPE_DOMAIN:         return create< std_msgs::String,  ObjectDict::DEFTYPE_DOMAIN         >(nh, name, s, key, force);
//
//         default: return 0;
//     }
// }

void RosChain::logState(const can::State & s)
{
  RCLCPP_INFO(this->get_logger(), "logState");
  can::DriverInterfaceSharedPtr interface = interface_;
  // std::string msg;
  // if(interface && !interface->translateError(s.internal_error, msg)) msg  =  "Undefined"; ;
  // ROS_INFO_STREAM("Current state: " << s.driver_state << " device error: " << s.error_code << " internal_error: " << s.internal_error << " (" << msg << ")");
}

// void RosChain::run(){
//     running_ = true;
//     time_point abs_time = boost::chrono::high_resolution_clock::now();
//     while(running_){
//         LayerStatus s;
//         try{
//             read(s);
//             write(s);
//             if(!s.bounded<LayerStatus::Warn>()) ROS_ERROR_STREAM_THROTTLE(10, s.reason());
//             else if(!s.bounded<LayerStatus::Ok>()) ROS_WARN_STREAM_THROTTLE(10, s.reason());
//         }
//         catch(const canopen::Exception& e){
//             ROS_ERROR_STREAM_THROTTLE(1, boost::diagnostic_information(e));
//         }
//         if(!sync_){
//             abs_time += update_duration_;
//             boost::this_thread::sleep_until(abs_time);
//         }
//     }
// }

// bool RosChain::handle_init(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
//     ROS_INFO("Initializing XXX");
//     boost::mutex::scoped_lock lock(mutex_);
//     if(getLayerState() > Off){
//         res.success = true;
//         res.message = "already initialized";
//         return true;
//     }
//     thread_.reset(new boost::thread(&RosChain::run, this));
//     LayerReport status;
//     try{
//         init(status);
//         res.success = status.bounded<LayerStatus::Ok>();
//         res.message = status.reason();
//         if(!status.bounded<LayerStatus::Warn>()){
//             diag(status);
//             res.message = status.reason();
//         }else{
//             heartbeat_timer_.restart();
//             return true;
//         }
//     }
//     catch( const std::exception &e){
//         std::string info = boost::diagnostic_information(e);
//         ROS_ERROR_STREAM(info);
//         res.message = info;
//         status.error(res.message);
//     }
//     catch(...){
//         res.message = "Unknown exception";
//         status.error(res.message);
//     }
//
//     res.success = false;
//     shutdown(status);
//
//     return true;
// }

// bool RosChain::handle_recover(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
//     ROS_INFO("Recovering XXX");
//     boost::mutex::scoped_lock lock(mutex_);
//     res.success = false;
//
//     if(getLayerState() > Init){
//         LayerReport status;
//         try{
//             if(!reset_errors_before_recover_ || emcy_handlers_->callFunc<LayerStatus::Warn>(&EMCYHandler::resetErrors, status)){
//                 recover(status);
//             }
//             if(!status.bounded<LayerStatus::Warn>()){
//                 diag(status);
//             }
//             res.success = status.bounded<LayerStatus::Warn>();
//             res.message = status.reason();
//         }
//         catch( const std::exception &e){
//             std::string info = boost::diagnostic_information(e);
//             ROS_ERROR_STREAM(info);
//             res.message = info;
//         }
//         catch(...){
//             res.message = "Unknown exception";
//         }
//     }else{
//         res.message = "not running";
//     }
//     return true;
// }

// void RosChain::handleWrite(LayerStatus &status, const LayerState &current_state) {
//     LayerStack::handleWrite(status, current_state);
//     if(current_state > Shutdown){
//         for(const PublishFuncType& func: publishers_) func();
//     }
// }

// void RosChain::handleShutdown(LayerStatus &status){
//     boost::mutex::scoped_lock lock(diag_mutex_);
//     heartbeat_timer_.stop();
//     LayerStack::handleShutdown(status);
//     if(running_){
//         running_ = false;
//         thread_->interrupt();
//         thread_->join();
//         thread_.reset();
//     }
// }

// bool RosChain::handle_shutdown(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
//     ROS_INFO("Shuting down XXX");
//     boost::mutex::scoped_lock lock(mutex_);
//     res.success = true;
//     if(getLayerState() > Init){
//         LayerStatus s;
//         halt(s);
//         shutdown(s);
//     }else{
//         res.message = "not running";
//     }
//     return true;
// }

// bool RosChain::handle_halt(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
//     ROS_INFO("Halting down XXX");
//     boost::mutex::scoped_lock lock(mutex_);
//      res.success = true;
//      if(getLayerState() > Init){
//         LayerStatus s;
//         halt(s);
//     }else{
//         res.message = "not running";
//     }
//     return true;
// }

// bool RosChain::handle_get_object(canopen_chain_node::GetObject::Request  &req, canopen_chain_node::GetObject::Response &res){
//     std::map<std::string, canopen::NodeSharedPtr >::iterator it = nodes_lookup_.find(req.node);
//     if(it == nodes_lookup_.end()){
//         res.message = "node not found";
//     }else{
//         try {
//             res.value = it->second->getStorage()->getStringReader(canopen::ObjectDict::Key(req.object), req.cached)();
//             res.success = true;
//         } catch(std::exception& e) {
//             res.message = boost::diagnostic_information(e);
//         }
//     }
//     return true;
// }

// bool RosChain::handle_set_object(canopen_chain_node::SetObject::Request  &req, canopen_chain_node::SetObject::Response &res){
//     std::map<std::string, canopen::NodeSharedPtr >::iterator it = nodes_lookup_.find(req.node);
//     if(it == nodes_lookup_.end()){
//         res.message = "node not found";
//     }else{
//         try {
//             it->second->getStorage()->getStringWriter(canopen::ObjectDict::Key(req.object), req.cached)(req.value);
//             res.success = true;
//         } catch(std::exception& e) {
//             res.message = boost::diagnostic_information(e);
//         }
//     }
//     return true;
// }

bool RosChain::setup_bus()
{
  RCLCPP_INFO(this->get_logger(), "setup_bus");

  std::string can_device;
  std::string driver_plugin;
  std::string master_alloc;
  bool loopback;

  if (!get_parameter_or("bus.device", can_device, std::string("can0"))) {
    RCLCPP_WARN(this->get_logger(),
      "CAN device not specified, using can0");
  }
  RCLCPP_INFO(this->get_logger(), "can: %s", can_device.c_str());

  if (!get_parameter_or("bus.driver_plugin", driver_plugin, std::string("can::SocketCANInterface"))) {
    RCLCPP_WARN(this->get_logger(),
      "driver_plugin not specified, using can::SocketCANInterface");
  }
  RCLCPP_INFO(this->get_logger(), "driver_plugin: %s", driver_plugin.c_str());

  if (!get_parameter_or("bus.loopback", loopback, false)) {
    RCLCPP_WARN(this->get_logger(),
      "loopback not specified, socket will not loop back messages");
  }
  RCLCPP_INFO(this->get_logger(), "loopback: %d", loopback);

  try {
    interface_ = driver_loader_.createInstance(driver_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), ex.what());
    return false;
  }

  state_listener_ = interface_->createStateListenerM(this, &RosChain::logState);

  if (!get_parameter_or("bus.master_allocator", master_alloc,
    std::string("canopen_master/SimpleMasterAllocator")))
  {
    RCLCPP_WARN(this->get_logger(),
      "master_allocator not specified, using canopen_master/SimpleMasterAllocator");
  }
  RCLCPP_INFO(this->get_logger(), "master_allocator: %s", master_alloc.c_str());

  try {
    master_ = master_allocator_.allocateInstance(master_alloc, can_device, interface_);
  } catch (const std::exception & e) {
    std::string info = boost::diagnostic_information(e);
    RCLCPP_ERROR(this->get_logger(), info);
    return false;
  }

  if (!master_) {
    RCLCPP_ERROR(this->get_logger(), "Could not allocate master");
    return false;
  }

  add(std::make_shared<CANLayer>(interface_, can_device, loopback));

  return true;
}

bool RosChain::setup_sync()
{
  RCLCPP_INFO(this->get_logger(), "setup_sync");
  int sync_ms;
  int sync_overflow;

  if (!get_parameter_or("sync.interval_ms", sync_ms, 0)) {
    RCLCPP_WARN(this->get_logger(),
      "sync interval was not specified, so sync is disabled per default");
  }
  RCLCPP_INFO(this->get_logger(), "interval_ms: %d", sync_ms);
  if (sync_ms < 0) {
    RCLCPP_ERROR(this->get_logger(),
      "sync interval " + std::to_string(sync_ms) + " is invalid");
    return false;
  }

  int update_ms = sync_ms;
  if (sync_ms == 0) {
    get_parameter_or("sync.update_ms", update_ms, 10);
    RCLCPP_INFO(this->get_logger(), "update_ms: %d", update_ms);
  }
  if (update_ms == 0) {
    RCLCPP_ERROR(this->get_logger(),
      "update interval " + std::to_string(update_ms) + " is invalid");
    return false;
  } else {
    update_duration_ = boost::chrono::milliseconds(update_ms);
  }

  if (sync_ms) {
    if (!get_parameter_or("sync.overflow", sync_overflow, 0)) {
      RCLCPP_WARN(this->get_logger(),
        "sync overflow was not specified, so overflow is disabled per default");
    }
    RCLCPP_INFO(this->get_logger(), "overflow: %d", sync_overflow);
    if (sync_overflow == 1 || sync_overflow > 240) {
      RCLCPP_ERROR(this->get_logger(),
        "sync overflow " + std::to_string(sync_overflow) + " is invalid");
      return false;
    }

    // TODO: parse header
    sync_ = master_->getSync(SyncProperties(can::MsgHeader(0x80), sync_ms, sync_overflow));

    if (!sync_ && sync_ms) {
      RCLCPP_ERROR(this->get_logger(),
        "Initializing sync master failed");
      return false;
    }
    add(sync_);
  }
  return true;
}

bool RosChain::setup_heartbeat()
{
  RCLCPP_INFO(this->get_logger(), "setup_heartbeat");

  std::string msg;
  double rate = 0;

  bool got_any = false;

  if (get_parameter_or("heartbeat.msg", msg, std::string("77f#05"))) {
    got_any = true;
  } else {
    RCLCPP_WARN(this->get_logger(),
      "heartbeat_msg not specified, using 77f#05");
  }
  RCLCPP_INFO(this->get_logger(), "heartbeat_msg: %s", msg.c_str());

  if (get_parameter_or("heartbeat.rate", rate, 10.0)) {
    got_any = true;
  } else {
    RCLCPP_WARN(this->get_logger(),
      "heartbeat_rate not specified, using 10.0");
  }
  RCLCPP_INFO(this->get_logger(), "heartbeat_rate: %f", rate);

  if (!got_any) {
    return true; // nothing todo
  }

  if (rate <= 0) {
    RCLCPP_ERROR(this->get_logger(),
      "rate " + std::to_string(rate) + " is invalid");
    return false;
  }

  hb_sender_.frame = can::toframe(msg);

  if (!hb_sender_.frame.isValid()) {
    // ROS_ERROR_STREAM("Message '"<< msg << "' is invalid");
    RCLCPP_ERROR(this->get_logger(),
      "heartbeat_msg " + msg + " is invalid");
    return false;
  }

  hb_sender_.interface = interface_;

  heartbeat_timer_.start(std::bind(&HeartbeatSender::send, &hb_sender_),
    boost::chrono::duration<double>(1.0 / rate), false);

  return true;
}

// std::pair<std::string, bool> parseObjectName(std::string obj_name){
//     size_t pos = obj_name.find('!');
//     bool force = pos != std::string::npos;
//     if(force) obj_name.erase(pos);
//     return std::make_pair(obj_name, force);
// }

// bool addLoggerEntries(XmlRpc::XmlRpcValue merged, const std::string param, uint8_t level, Logger &logger){
//     if(merged.hasMember(param)){
//         try{
//             XmlRpc::XmlRpcValue objs = merged[param];
//             for(int i = 0; i < objs.size(); ++i){
//                 std::pair<std::string, bool> obj_name = parseObjectName(objs[i]);
//
//                 if(!logger.add(level, obj_name.first, obj_name.second)){
//                     ROS_ERROR_STREAM("Could not create logger for '" << obj_name.first << "'");
//                     return false;
//                 }
//             }
//         }
//         catch(...){
//             ROS_ERROR_STREAM("Could not parse " << param << " parameter");
//             return false;
//         }
//     }
//     return true;
// }

bool RosChain::setup_nodes()
{
  RCLCPP_INFO(this->get_logger(), "setup_heartbeat");
  nodes_.reset(new canopen::LayerGroupNoDiag<canopen::Node>("301 layer"));
  add(nodes_);

  emcy_handlers_.reset(new canopen::LayerGroupNoDiag<canopen::EMCYHandler>("EMCY layer"));

  // XmlRpc::XmlRpcValue nodes;
  std::vector<std::string> nodes;
  if (!get_parameter_or("nodes", nodes, {})) {
    RCLCPP_WARN(this->get_logger(),
      "no nodes!");
    return false;
  }

  // MergedXmlRpcStruct defaults;
  // nh_priv_.getParam("defaults", defaults);
  //
  // if(nodes.getType() ==  XmlRpc::XmlRpcValue::TypeArray){
  //     XmlRpc::XmlRpcValue new_stuct;
  //     for(size_t i = 0; i < nodes.size(); ++i){
  //         if(nodes[i].hasMember("name")){
  //             std::string &name = nodes[i]["name"];
  //             new_stuct[name] = nodes[i];
  //         }else{
  //             ROS_ERROR_STREAM("Node at list index " << i << " has no name");
  //             return false;
  //         }
  //     }
  //     nodes = new_stuct;
  // }
  //
  // for(XmlRpc::XmlRpcValue::iterator it = nodes.begin(); it != nodes.end(); ++it){
  //     int node_id;
  //     try{
  //         node_id = it->second["id"];
  //     }
  //     catch(...){
  //         ROS_ERROR_STREAM("Node '" << it->first  << "' has no id");
  //         return false;
  //     }
  //     MergedXmlRpcStruct merged(it->second, defaults);
  //
  //     if(!it->second.hasMember("name")){
  //         merged["name"]=it->first;
  //     }
  //
  //     ObjectDict::Overlay overlay;
  //     if(merged.hasMember("dcf_overlay")){
  //         XmlRpc::XmlRpcValue dcf_overlay = merged["dcf_overlay"];
  //         if(dcf_overlay.getType() != XmlRpc::XmlRpcValue::TypeStruct){
  //             ROS_ERROR_STREAM("dcf_overlay is no struct");
  //             return false;
  //         }
  //         for(XmlRpc::XmlRpcValue::iterator ito = dcf_overlay.begin(); ito!= dcf_overlay.end(); ++ito){
  //             if(ito->second.getType() != XmlRpc::XmlRpcValue::TypeString){
  //                 ROS_ERROR_STREAM("dcf_overlay '" << ito->first << "' must be string");
  //                 return false;
  //             }
  //             overlay.push_back(ObjectDict::Overlay::value_type(ito->first, ito->second));
  //         }
  //
  //     }
  //
  //     std::string eds;
  //
  //     try{
  //         eds = (std::string) merged["eds_file"];
  //     }
  //     catch(...){
  //         ROS_ERROR_STREAM("EDS path '" << eds << "' invalid");
  //         return false;
  //     }
  //
  //     try{
  //         if(merged.hasMember("eds_pkg")){
  //             std::string pkg = merged["eds_pkg"];
  //             std::string p = ros::package::getPath(pkg);
  //             if(p.empty()){
  //                     ROS_WARN_STREAM("Package '" << pkg << "' was not found");
  //             }else{
  //                 eds = (boost::filesystem::path(p)/eds).make_preferred().native();;
  //             }
  //         }
  //     }
  //     catch(...){
  //     }
  //
  //     ObjectDictSharedPtr  dict = ObjectDict::fromFile(eds, overlay);
  //     if(!dict){
  //         ROS_ERROR_STREAM("EDS '" << eds << "' could not be parsed");
  //         return false;
  //     }
  //     canopen::NodeSharedPtr node = std::make_shared<canopen::Node>(interface_, dict, node_id, sync_);
  //
  //     LoggerSharedPtr logger = std::make_shared<Logger>(node);
  //
  //     if(!nodeAdded(merged, node, logger)) return false;
  //
  //     if(!addLoggerEntries(merged, "log", diagnostic_updater::DiagnosticStatusWrapper::OK, *logger)) return false;
  //     if(!addLoggerEntries(merged, "log_warn", diagnostic_updater::DiagnosticStatusWrapper::WARN, *logger)) return false;
  //     if(!addLoggerEntries(merged, "log_error", diagnostic_updater::DiagnosticStatusWrapper::ERROR, *logger)) return false;
  //
  //     loggers_.push_back(logger);
  //     diag_updater_.add(it->first, std::bind(&Logger::log, logger, std::placeholders::_1));
  //
  //     std::string node_name = std::string(merged["name"]);
  //
  //     if(merged.hasMember("publish")){
  //         try{
  //             XmlRpc::XmlRpcValue objs = merged["publish"];
  //             for(int i = 0; i < objs.size(); ++i){
  //                 std::pair<std::string, bool> obj_name = parseObjectName(objs[i]);
  //
  //                 PublishFuncType pub = createPublishFunc(nh_, node_name +"_"+obj_name.first, node, obj_name.first, obj_name.second);
  //                 if(!pub){
  //                     ROS_ERROR_STREAM("Could not create publisher for '" << obj_name.first << "'");
  //                     return false;
  //                 }
  //                 publishers_.push_back(pub);
  //             }
  //         }
  //         catch(...){
  //             ROS_ERROR("Could not parse publish parameter");
  //             return false;
  //         }
  //     }
  //     nodes_->add(node);
  //     nodes_lookup_.insert(std::make_pair(node_name, node));
  //
  //     std::shared_ptr<canopen::EMCYHandler> emcy = std::make_shared<canopen::EMCYHandler>(interface_, node->getStorage());
  //     emcy_handlers_->add(emcy);
  //     logger->add(emcy);
  //
  // }
  return true;
}

// bool RosChain::nodeAdded(XmlRpc::XmlRpcValue &params, const canopen::NodeSharedPtr &node, const LoggerSharedPtr &logger){
//     return true;
// }

void RosChain::report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  RCLCPP_INFO(this->get_logger(), "report_diagnostics");
  // boost::mutex::scoped_lock lock(diag_mutex_);
  // LayerReport r;
  // if(getLayerState() == Off){
  //     stat.summary(stat.WARN,"Not initailized");
  // }else if(!running_){
  //     stat.summary(stat.ERROR,"Thread is not running");
  // }else{
  //     diag(r);
  //     if(r.bounded<LayerStatus::Unbounded>()){ // valid
  //         stat.summary(r.get(), r.reason());
  //         for(std::vector<std::pair<std::string, std::string> >::const_iterator it = r.values().begin(); it != r.values().end(); ++it){
  //             stat.add(it->first, it->second);
  //         }
  //     }
  // }
}

RosChain::RosChain()
: Node("canopen_chain_node"),
  LayerStack("ROS stack"),
  driver_loader_("socketcan_interface", "can::DriverInterface"),
  master_allocator_("canopen_master", "canopen::Master::Allocator"),
  running_(false),
  reset_errors_before_recover_(false)
{
  RCLCPP_INFO(this->get_logger(), "constructor");
}

bool RosChain::setup()
{
  RCLCPP_INFO(this->get_logger(), "setup");
  boost::mutex::scoped_lock lock(mutex_);
  bool okay = setup_chain();
  if (okay) {add(emcy_handlers_);}
  return okay;
}

bool RosChain::setup_chain()
{
  RCLCPP_INFO(this->get_logger(), "setup_chain");

  std::string hardware_id;
  get_parameter_or_set("hardware_id", hardware_id, std::string("none"));
  get_parameter_or_set("reset_errors_before_recover",
    reset_errors_before_recover_, false);

  RCLCPP_INFO(this->get_logger(), "hardware_id: %s", hardware_id.c_str());
  RCLCPP_INFO(this->get_logger(), "reset_errors_before_recover: %d", reset_errors_before_recover_);

  diag_updater_.setHardwareID(hardware_id);
  diag_updater_.add("chain", this, &RosChain::report_diagnostics);
  diag_timer_ =
    this->create_wall_timer(1s, std::bind(&diagnostic_updater::Updater::update, &diag_updater_));

  // ros::NodeHandle nh_driver(nh_, "driver");
  //
  // srv_init_ = nh_driver.advertiseService("init",&RosChain::handle_init, this);
  // srv_recover_ = nh_driver.advertiseService("recover",&RosChain::handle_recover, this);
  // srv_halt_ = nh_driver.advertiseService("halt",&RosChain::handle_halt, this);
  // srv_shutdown_ = nh_driver.advertiseService("shutdown",&RosChain::handle_shutdown, this);
  //
  // srv_get_object_ = nh_driver.advertiseService("get_object",&RosChain::handle_get_object, this);
  // srv_set_object_ = nh_driver.advertiseService("set_object",&RosChain::handle_set_object, this);

  return setup_bus() && setup_sync() && setup_heartbeat() && setup_nodes();
  // return setup_bus() && setup_sync() && setup_heartbeat();
  // return setup_bus() && setup_heartbeat();
}

RosChain::~RosChain()
{
  try {
    LayerStatus s;
    halt(s);
    shutdown(s);
  } catch (...) {
    LOG("CATCH");
  }
}

}
