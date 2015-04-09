#ifndef H_CANOPEN_CHAIN_ROS
#define H_CANOPEN_CHAIN_ROS

#include <canopen_master/canopen.h>
#include <canopen_master/master.h>
#include <canopen_master/can_layer.h>
#include <socketcan_interface/string.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cob_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/filesystem/path.hpp>
#include <boost/weak_ptr.hpp>
#include <pluginlib/class_loader.h>

namespace canopen{

class MergedXmlRpcStruct : public XmlRpc::XmlRpcValue{
    MergedXmlRpcStruct(const XmlRpc::XmlRpcValue& a) :XmlRpc::XmlRpcValue(a){ assertStruct(); }
public:
    MergedXmlRpcStruct(){ assertStruct(); }
    MergedXmlRpcStruct(const XmlRpc::XmlRpcValue& a, const MergedXmlRpcStruct &b, bool recursive= true) :XmlRpc::XmlRpcValue(a){
        assertStruct();

        for(ValueStruct::const_iterator it = b._value.asStruct->begin(); it != b._value.asStruct->end(); ++it){
            std::pair<XmlRpc::XmlRpcValue::iterator,bool> res =  _value.asStruct->insert(*it);

            if(recursive && !res.second && res.first->second.getType() == XmlRpc::XmlRpcValue::TypeStruct && it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct){
                res.first->second = MergedXmlRpcStruct(res.first->second, it->second); // recursive struct merge with implicit cast
            }
        }


    }
};

class Logger: public DiagGroup<canopen::Layer>{
    const boost::shared_ptr<canopen::Node> node_;
    
    std::vector<boost::function< void (diagnostic_updater::DiagnosticStatusWrapper &)> > entries_;
    
    template<typename T> void log_entry(diagnostic_updater::DiagnosticStatusWrapper &stat, const std::string &name, const ObjectDict::Key &key){
        stat.add(name, node_->template get<T>(key));
    }

public:
    Logger(boost::shared_ptr<canopen::Node> node):  node_(node) { add(node_); }
    
    template<typename T> void add(const std::string &name, const ObjectDict::Key &key){
            entries_.push_back(boost::bind(&Logger::log_entry<T>, this, _1, name, key));
    }
    template<const uint16_t dt> static void func(Logger &l, const std::string &n, const ObjectDict::Key &k){
        l.template add<typename ObjectStorage::DataType<dt>::type>(n,k);
    }
    void add(const uint16_t data_type, const std::string &name, const ObjectDict::Key &key){
        branch_type<Logger, void (Logger &, const std::string &, const ObjectDict::Key &)>(data_type)(*this,name, key);
    }

    template<typename T> void add(const boost::shared_ptr<T> &n){
        DiagGroup::add(boost::static_pointer_cast<canopen::Layer>(n));
    }

    virtual void log(diagnostic_updater::DiagnosticStatusWrapper &stat){
        if(node_->getState() == canopen::Node::Unknown){
            stat.summary(stat.WARN,"Not initailized");
        }else{
            LayerReport r;
            diag(r);
            if(r.bounded<LayerStatus::Unbounded>()){ // valid
                stat.summary(r.get(), r.reason());
                for(std::vector<std::pair<std::string, std::string> >::const_iterator it = r.values().begin(); it != r.values().end(); ++it){
                    stat.add(it->first, it->second);
                }
            }
        }
        // for(size_t i=0; i < entries_.size(); ++i) entries_[i](stat); TODO
    }
    virtual ~Logger() {}
};

class RosChain : public canopen::LayerStack {
      pluginlib::ClassLoader<can::DriverInterface> driver_loader_;
protected:
    boost::shared_ptr<can::DriverInterface> interface_;
    boost::shared_ptr<Master> master_;
    boost::shared_ptr<canopen::LayerGroupNoDiag<canopen::Node> > nodes_;
    boost::shared_ptr<canopen::SyncLayer> sync_;
    std::vector<boost::shared_ptr<Logger > > loggers_;


    can::StateInterface::StateListener::Ptr state_listener_;
    
    boost::scoped_ptr<boost::thread> thread_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    
    diagnostic_updater::Updater diag_updater_;
    ros::Timer diag_timer_;

    boost::mutex mutex_;
    ros::ServiceServer srv_init_;
    ros::ServiceServer srv_recover_;
    ros::ServiceServer srv_halt_;
    ros::ServiceServer srv_shutdown_;

    time_duration update_duration_;

    ros::Timer heartbeat_timer_;

    
    void logState(const can::State &s){
        boost::shared_ptr<can::DriverInterface> interface = interface_;
        std::string msg;
        if(interface && !interface->translateError(s.internal_error, msg)) msg  =  "Undefined"; ;
        ROS_INFO_STREAM("Current state: " << s.driver_state << " device error: " << s.error_code << " internal_error: " << s.internal_error << " (" << msg << ")");
    }
    
    void run(){

        time_point abs_time = boost::chrono::high_resolution_clock::now();
        while(ros::ok()){
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
    
    virtual bool handle_init(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
	ROS_INFO("Initializing XXX");
        boost::mutex::scoped_lock lock(mutex_);
        if(thread_){
            res.success.data = true;
            res.error_message.data = "already initialized";
            return true;
        }
        thread_.reset(new boost::thread(&RosChain::run, this));
        LayerReport status;
        try{
            init(status);
            res.success.data = status.bounded<LayerStatus::Ok>();
            res.error_message.data = status.reason();
            if(!status.bounded<LayerStatus::Warn>()){
                diag(status);
                shutdown(status);
                thread_.reset();
            }else{
                heartbeat_timer_.start();
            }
        }
        catch( const canopen::Exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            res.success.data = false;
            res.error_message.data = info;
            status.error(info);
            shutdown(status);
            thread_.reset();
        }
        return true;
    }
    virtual bool handle_recover(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
	ROS_INFO("Recovering XXX");
        boost::mutex::scoped_lock lock(mutex_);
        if(thread_){
            LayerReport status;
            try{
                thread_->interrupt();
                thread_->join();
                thread_.reset(new boost::thread(&RosChain::run, this));
                recover(status);
                if(!status.bounded<LayerStatus::Warn>()){
                    diag(status);
                }
                res.success.data = status.bounded<LayerStatus::Warn>();
                res.error_message.data = status.reason();
            }
            catch( const canopen::Exception &e){
                std::string info = boost::diagnostic_information(e);
                ROS_ERROR_STREAM(info);
                res.success.data = false;
                res.error_message.data = info;
            }
        }else{
            res.success.data = false;
            res.error_message.data = "not running";
        }
        return true;
    }
    virtual void handleShutdown(LayerStatus &status){
        heartbeat_timer_.stop();
        if(thread_){
            halt(status);
            thread_->interrupt();
            thread_->join();
            LayerStack::handleShutdown(status);
            thread_.reset();
        }
    }

    virtual bool handle_shutdown(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
	ROS_INFO("Shuting down XXX");
        boost::mutex::scoped_lock lock(mutex_);
        res.success.data = true;
        if(thread_){
            LayerStatus s;
            shutdown(s);
        }else{
            res.error_message.data = "not running";
        }
        return true;
    }
    virtual bool handle_halt(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
	ROS_INFO("Halting down XXX");
        boost::mutex::scoped_lock lock(mutex_);
         res.success.data = true;
         if(thread_){
            LayerStatus s;
            halt(s);
        }else{
            res.error_message.data = "not running";
        }
        return true;
    }
    bool setup_bus(){
        ros::NodeHandle bus_nh(nh_priv_,"bus");
        std::string can_device;
        std::string driver_plugin;
        std::string master_type;
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
        
        bus_nh.param("master_type",master_type, std::string("shared"));

        try{
            if(master_type == "exclusive"){
                master_ = boost::make_shared<SharedMaster>(can_device, interface_);
            }else if (master_type == "shared"){
                boost::interprocess::permissions perm;
                perm.set_unrestricted();
                master_ = boost::make_shared<SharedMaster>(can_device, interface_, perm);
            }else if (master_type == "local"){
                master_ = boost::make_shared<LocalMaster>(can_device, interface_);
            }else{
                ROS_ERROR_STREAM("Master type  "<< master_type << " is not supported");
                return false;
            }
        }
        catch( const std::exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            return false;
        }
        
        add(boost::make_shared<CANLayer>(interface_, can_device, loopback));
        
        return true;
    }
    bool setup_sync(){
        ros::NodeHandle sync_nh(nh_priv_,"sync");
        
        int sync_ms = 0;
        int silence_us = 0;
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
            sync_nh.getParam("silence_us", silence_us);

            // TODO: parse header
            sync_ = master_->getSync(SyncProperties(can::MsgHeader(0x80), boost::posix_time::milliseconds(sync_ms), boost::posix_time::microseconds(silence_us), sync_overflow));
            
            if(!sync_ && sync_ms){
                ROS_ERROR_STREAM("Initializing sync master failed");
                return false;
            }
            add(sync_);
        }
        return true;
    }
    bool setup_heartbeat(){
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

            can::Frame frame = can::toframe(msg);


            if(!frame.isValid()){
                ROS_ERROR_STREAM("Message '"<< msg << "' is invalid");
                return false;
            }

            heartbeat_timer_ = hb_nh.createTimer(ros::Duration(1.0/rate), boost::bind(&can::DriverInterface::send,interface_, frame), false, false);

            return true;


    }
    bool setup_nodes(){
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
            
            nodes_->add(node);
        }
        return true;
    }
    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger) { return true; }
    void report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
        LayerReport r;
        if(!thread_){
            stat.summary(stat.WARN,"Not initailized");
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
public:
    RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): LayerStack("ROS stack"),driver_loader_("socketcan_interface", "can::DriverInterface"),nh_(nh), nh_priv_(nh_priv), diag_updater_(nh_,nh_priv_){}
    virtual bool setup(){
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
    virtual ~RosChain(){
        try{
            LayerStatus s;
            shutdown(s);
        }catch(...){ }
        destroy();
    }
};

} //namespace canopen

#endif

