#include <ros/package.h>

#include <canopen_chain_node/ros_chain.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

namespace canopen {

PublishFunc::func_type PublishFunc::create(ros::NodeHandle &nh,  const std::string &name, boost::shared_ptr<canopen::Node> node, const std::string &key, bool force){
    boost::shared_ptr<ObjectStorage> s = node->getStorage();

    switch(ObjectDict::DataTypes(s->dict_->get(key)->data_type)){
        case ObjectDict::DEFTYPE_INTEGER8:       return create< std_msgs::Int8    >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER8>::type>(key), force);
        case ObjectDict::DEFTYPE_INTEGER16:      return create< std_msgs::Int16   >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER16>::type>(key), force);
        case ObjectDict::DEFTYPE_INTEGER32:      return create< std_msgs::Int32   >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER32>::type>(key), force);
        case ObjectDict::DEFTYPE_INTEGER64:      return create< std_msgs::Int64   >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER64>::type>(key), force);

        case ObjectDict::DEFTYPE_UNSIGNED8:      return create< std_msgs::UInt8   >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED8>::type>(key), force);
        case ObjectDict::DEFTYPE_UNSIGNED16:     return create< std_msgs::UInt16  >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16>::type>(key), force);
        case ObjectDict::DEFTYPE_UNSIGNED32:     return create< std_msgs::UInt32  >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED32>::type>(key), force);
        case ObjectDict::DEFTYPE_UNSIGNED64:     return create< std_msgs::UInt64  >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED64>::type>(key), force);

        case ObjectDict::DEFTYPE_REAL32:         return create< std_msgs::Float32 >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_REAL32>::type>(key), force);
        case ObjectDict::DEFTYPE_REAL64:         return create< std_msgs::Float64 >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_REAL64>::type>(key), force);

        case ObjectDict::DEFTYPE_VISIBLE_STRING: return create< std_msgs::String  >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_VISIBLE_STRING>::type>(key), force);
        case ObjectDict::DEFTYPE_OCTET_STRING:   return create< std_msgs::String  >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_DOMAIN>::type>(key), force);
        case ObjectDict::DEFTYPE_UNICODE_STRING: return create< std_msgs::String  >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNICODE_STRING>::type>(key), force);
        case ObjectDict::DEFTYPE_DOMAIN:         return create< std_msgs::String  >(nh, name, s->entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_DOMAIN>::type>(key), force);

        default: return 0;
    }
}

void RosChain::logState(const can::State &s){
    boost::shared_ptr<can::DriverInterface> interface = interface_;
    std::string msg;
    if(interface && !interface->translateError(s.internal_error, msg)) msg  =  "Undefined"; ;
    ROS_INFO_STREAM("Current state: " << s.driver_state << " device error: " << s.error_code << " internal_error: " << s.internal_error << " (" << msg << ")");
}

void RosChain::run(){

    time_point abs_time = boost::chrono::high_resolution_clock::now();
    while(true){
        LayerStatus s;
        try{
            read(s);
            write(s);
            if(!s.bounded<LayerStatus::Warn>()) ROS_ERROR_STREAM_THROTTLE(10, s.reason());
            else if(!s.bounded<LayerStatus::Ok>()) ROS_WARN_STREAM_THROTTLE(10, s.reason());
        }
        catch(const canopen::Exception& e){
            ROS_ERROR_STREAM_THROTTLE(1, boost::diagnostic_information(e));
        }
        abs_time += update_duration_;

        boost::this_thread::sleep_until(abs_time);
    }
}

bool RosChain::handle_init(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
    ROS_INFO("Initializing XXX");
    boost::mutex::scoped_lock lock(mutex_);
    if(initialized_){
        res.success = true;
        res.message = "already initialized";
        return true;
    }
    thread_.reset(new boost::thread(&RosChain::run, this));
    LayerReport status;
    try{
        init(status);
        initialized_ = true;
        res.success = status.bounded<LayerStatus::Ok>();
        res.message = status.reason();
        if(!status.bounded<LayerStatus::Warn>()){
            diag(status);
            res.message = status.reason();
        }else{
            heartbeat_timer_.restart();
            return true;
        }
    }
    catch( const std::exception &e){
        std::string info = boost::diagnostic_information(e);
        ROS_ERROR_STREAM(info);
        res.message = info;
        status.error(res.message);
    }
    catch(...){
        res.message = "Unknown exception";
        status.error(res.message);
    }

    res.success = false;
    shutdown(status);
    thread_.reset();
    initialized_ = false;

    return true;
}
bool RosChain::handle_recover(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
    ROS_INFO("Recovering XXX");
    boost::mutex::scoped_lock lock(mutex_);
    res.success = false;

    if(initialized_ && thread_){
        LayerReport status;
        try{
            thread_->interrupt();
            thread_->join();
            thread_.reset(new boost::thread(&RosChain::run, this));
            recover(status);
            if(!status.bounded<LayerStatus::Warn>()){
                diag(status);
            }
            res.success = status.bounded<LayerStatus::Warn>();
            res.message = status.reason();
        }
        catch( const std::exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            res.message = info;
        }
        catch(...){
            res.message = "Unknown exception";
        }
    }else{
        res.message = "not running";
    }
    return true;
}

void RosChain::handleWrite(LayerStatus &status, const LayerState &current_state) {
    LayerStack::handleWrite(status, current_state);
    if(current_state > Shutdown){
        for(std::vector<boost::function<void() > >::iterator it = publishers_.begin(); it != publishers_.end(); ++it) (*it)();
    }
}

void RosChain::handleShutdown(LayerStatus &status){
    boost::mutex::scoped_lock lock(diag_mutex_);
    heartbeat_timer_.stop();
    LayerStack::handleShutdown(status);
    if(initialized_ &&  thread_){
        thread_->interrupt();
        thread_->join();
        thread_.reset();
    }
    initialized_ = false;
}

bool RosChain::handle_shutdown(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
    ROS_INFO("Shuting down XXX");
    boost::mutex::scoped_lock lock(mutex_);
    res.success = true;
    if(initialized_ && thread_){
        LayerStatus s;
        halt(s);
        shutdown(s);
    }else{
        res.message = "not running";
    }
    return true;
}

bool RosChain::handle_halt(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
    ROS_INFO("Halting down XXX");
    boost::mutex::scoped_lock lock(mutex_);
     res.success = true;
     if(initialized_){
        LayerStatus s;
        halt(s);
    }else{
        res.message = "not running";
    }
    return true;
}

bool RosChain::setup_bus(){
    ros::NodeHandle bus_nh(nh_priv_,"bus");
    std::string can_device;
    std::string driver_plugin;
    std::string master_alloc;
    bool loopback;

    if(!bus_nh.getParam("device",can_device)){
        ROS_ERROR("Device not set");
        return false;
    }

    bus_nh.param("loopback",loopback, false);

    bus_nh.param("driver_plugin",driver_plugin, std::string("can::SocketCANInterface"));

    try{
        interface_ =  driver_loader_.createInstance(driver_plugin);
    }

    catch(pluginlib::PluginlibException& ex){
        ROS_ERROR_STREAM(ex.what());
        return false;
    }

    state_listener_ = interface_->createStateListener(can::StateInterface::StateDelegate(this, &RosChain::logState));

    if(bus_nh.getParam("master_type",master_alloc)){
        ROS_ERROR("please migrate to master allocators");
        return false;
    }

    bus_nh.param("master_allocator",master_alloc, std::string("canopen::SimpleMaster::Allocator"));

    try{
        master_= master_allocator_.allocateInstance(master_alloc, can_device, interface_);
    }
    catch( const std::exception &e){
        std::string info = boost::diagnostic_information(e);
        ROS_ERROR_STREAM(info);
        return false;
    }

    if(!master_){
        ROS_ERROR_STREAM("Could not allocate master.");
        return false;
    }

    add(boost::make_shared<CANLayer>(interface_, can_device, loopback));

    return true;
}

bool RosChain::setup_sync(){
    ros::NodeHandle sync_nh(nh_priv_,"sync");

    int sync_ms = 0;
    int sync_overflow = 0;

    if(!sync_nh.getParam("interval_ms", sync_ms)){
        ROS_WARN("Sync interval was not specified, so sync is disabled per default");
    }

    if(sync_ms < 0){
        ROS_ERROR_STREAM("Sync interval  "<< sync_ms << " is invalid");
        return false;
    }

    int update_ms = sync_ms;
    if(sync_ms == 0) nh_priv_.getParam("update_ms", update_ms);
    if(update_ms == 0){
        ROS_ERROR_STREAM("Update interval  "<< sync_ms << " is invalid");
        return false;
    }else{
        update_duration_ = boost::chrono::milliseconds(update_ms);
    }

    if(sync_ms){
        if(!sync_nh.getParam("overflow", sync_overflow)){
            ROS_WARN("Sync overflow was not specified, so overflow is disabled per default");
        }
        if(sync_overflow == 1 || sync_overflow > 240){
            ROS_ERROR_STREAM("Sync overflow  "<< sync_overflow << " is invalid");
            return false;
        }
        if(sync_nh.param("silence_us", 0) != 0){
            ROS_WARN("silence_us is not supported anymore");
        }

        // TODO: parse header
        sync_ = master_->getSync(SyncProperties(can::MsgHeader(0x80), sync_ms, sync_overflow));

        if(!sync_ && sync_ms){
            ROS_ERROR_STREAM("Initializing sync master failed");
            return false;
        }
        add(sync_);
    }
    return true;
}

bool RosChain::setup_heartbeat(){
        ros::NodeHandle hb_nh(nh_priv_,"heartbeat");
        std::string msg;
        double rate = 0;

        bool got_any = hb_nh.getParam("msg", msg);
        got_any = hb_nh.getParam("rate", rate) || got_any;

        if( !got_any) return true; // nothing todo

        if(rate <=0 ){
            ROS_ERROR_STREAM("Rate '"<< rate << "' is invalid");
            return false;
        }

        hb_sender_.frame = can::toframe(msg);


        if(!hb_sender_.frame.isValid()){
            ROS_ERROR_STREAM("Message '"<< msg << "' is invalid");
            return false;
        }

        hb_sender_.interface = interface_;

        heartbeat_timer_.start(Timer::TimerDelegate(&hb_sender_, &HeartbeatSender::send) , boost::chrono::duration<double>(1.0/rate), false);

        return true;


}
bool RosChain::setup_nodes(){
    nodes_.reset(new canopen::LayerGroupNoDiag<canopen::Node>("301 layer"));
    add(nodes_);

    XmlRpc::XmlRpcValue nodes;
    if(!nh_priv_.getParam("nodes", nodes)){
        ROS_WARN("falling back to 'modules', please switch to 'nodes'");
        nh_priv_.getParam("modules", nodes);
    }
    MergedXmlRpcStruct defaults;
    nh_priv_.getParam("defaults", defaults);

    if(nodes.getType() ==  XmlRpc::XmlRpcValue::TypeArray){
        XmlRpc::XmlRpcValue new_stuct;
        for(size_t i = 0; i < nodes.size(); ++i){
            if(nodes[i].hasMember("name")){
                std::string &name = nodes[i]["name"];
                new_stuct[name] = nodes[i];
            }else{
                ROS_ERROR_STREAM("Node at list index " << i << " has no name");
                return false;
            }
        }
        nodes = new_stuct;
    }

    for(XmlRpc::XmlRpcValue::iterator it = nodes.begin(); it != nodes.end(); ++it){
        int node_id;
        try{
            node_id = it->second["id"];
        }
        catch(...){
            ROS_ERROR_STREAM("Node '" << it->first  << "' has no id");
            return false;
        }
        MergedXmlRpcStruct merged(it->second, defaults);

        if(!it->second.hasMember("name")){
            merged["name"]=it->first;
        }

        ObjectDict::Overlay overlay;
        if(merged.hasMember("dcf_overlay")){
            XmlRpc::XmlRpcValue dcf_overlay = merged["dcf_overlay"];
            if(dcf_overlay.getType() != XmlRpc::XmlRpcValue::TypeStruct){
                ROS_ERROR_STREAM("dcf_overlay is no struct");
                return false;
            }
            for(XmlRpc::XmlRpcValue::iterator ito = dcf_overlay.begin(); ito!= dcf_overlay.end(); ++ito){
                if(ito->second.getType() != XmlRpc::XmlRpcValue::TypeString){
                    ROS_ERROR_STREAM("dcf_overlay '" << ito->first << "' must be string");
                    return false;
                }
                overlay.push_back(ObjectDict::Overlay::value_type(ito->first, ito->second));
            }

        }

        std::string eds;

        try{
            eds = (std::string) merged["eds_file"];
        }
        catch(...){
            ROS_ERROR_STREAM("EDS path '" << eds << "' invalid");
            return false;
        }

        try{
            if(merged.hasMember("eds_pkg")){
                std::string pkg = merged["eds_pkg"];
                std::string p = ros::package::getPath(pkg);
                if(p.empty()){
                        ROS_WARN_STREAM("Package '" << pkg << "' was not found");
                }else{
                    eds = (boost::filesystem::path(p)/eds).make_preferred().native();;
                }
            }
        }
        catch(...){
        }

        boost::shared_ptr<ObjectDict>  dict = ObjectDict::fromFile(eds, overlay);
        if(!dict){
            ROS_ERROR_STREAM("EDS '" << eds << "' could not be parsed");
            return false;
        }
        boost::shared_ptr<canopen::Node> node = boost::make_shared<canopen::Node>(interface_, dict, node_id, sync_);

        boost::shared_ptr<Logger> logger = boost::make_shared<Logger>(node);

        if(!nodeAdded(merged, node, logger)) return false;

        //logger->add(4,"pos", canopen::ObjectDict::Key(0x6064));
        loggers_.push_back(logger);
        diag_updater_.add(it->first, boost::bind(&Logger::log, logger, _1));

        if(merged.hasMember("publish")){
            try{
                XmlRpc::XmlRpcValue objs = merged["publish"];
                for(int i = 0; i < objs.size(); ++i){
                    std::string obj_name = objs[i];
                    size_t pos = obj_name.find('!');
                    bool force = pos != std::string::npos;
                    if(force) obj_name.erase(pos);

                    boost::function<void()> pub = PublishFunc::create(nh_, std::string(merged["name"])+"_"+obj_name, node, obj_name, force);
                    if(!pub){
                        ROS_ERROR_STREAM("Could not create publisher for '" << obj_name << "'");
                        return false;
                    }
                    publishers_.push_back(pub);
                }
            }
            catch(...){
                ROS_ERROR("Could not parse publish parameter");
                return false;
            }
        }
        nodes_->add(node);
    }
    return true;
}
bool RosChain::nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger){
    return true;
}

void RosChain::report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
    boost::mutex::scoped_lock lock(diag_mutex_);
    LayerReport r;
    if(!initialized_){
        stat.summary(stat.WARN,"Not initailized");
    }else if(!thread_){
        stat.summary(stat.ERROR,"Rhread was not created");
    }else{
        diag(r);
        if(r.bounded<LayerStatus::Unbounded>()){ // valid
            stat.summary(r.get(), r.reason());
            for(std::vector<std::pair<std::string, std::string> >::const_iterator it = r.values().begin(); it != r.values().end(); ++it){
                stat.add(it->first, it->second);
            }
        }
    }
}

RosChain::RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
: LayerStack("ROS stack"),driver_loader_("socketcan_interface", "can::DriverInterface"), master_allocator_("canopen_master", "canopen::Master::Allocator"),nh_(nh), nh_priv_(nh_priv), diag_updater_(nh_,nh_priv_), initialized_(false){}

bool RosChain::setup(){
    boost::mutex::scoped_lock lock(mutex_);

    std::string hw_id;
    nh_priv_.param("hardware_id", hw_id, std::string("none"));

    diag_updater_.setHardwareID(hw_id);
    diag_updater_.add("chain", this, &RosChain::report_diagnostics);

    diag_timer_ = nh_.createTimer(ros::Duration(diag_updater_.getPeriod()/2.0),boost::bind(&diagnostic_updater::Updater::update, &diag_updater_));

    ros::NodeHandle nh_driver(nh_, "driver");

    srv_init_ = nh_driver.advertiseService("init",&RosChain::handle_init, this);
    srv_recover_ = nh_driver.advertiseService("recover",&RosChain::handle_recover, this);
    srv_halt_ = nh_driver.advertiseService("halt",&RosChain::handle_halt, this);
    srv_shutdown_ = nh_driver.advertiseService("shutdown",&RosChain::handle_shutdown, this);

    return setup_bus() && setup_sync() && setup_heartbeat() && setup_nodes();
}

RosChain::~RosChain(){
    try{
        LayerStatus s;
        halt(s);
        shutdown(s);
    }catch(...){ LOG("CATCH"); }
}

}
