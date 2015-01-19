#ifndef H_CANOPEN_CHAIN_ROS
#define H_CANOPEN_CHAIN_ROS

#include <canopen_master/canopen.h>
#include <canopen_master/master.h>
#include <canopen_master/can_layer.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cob_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/filesystem/path.hpp>
#include <boost/weak_ptr.hpp>

namespace canopen{

class MergedXmlRpcStruct : public XmlRpc::XmlRpcValue{
public:
    MergedXmlRpcStruct(){
        assertStruct();
    }
    MergedXmlRpcStruct(XmlRpc::XmlRpcValue& a, MergedXmlRpcStruct &b) :XmlRpc::XmlRpcValue(a){
        assertStruct();
        _value.asStruct->insert(b._value.asStruct->begin(), b._value.asStruct->end());
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
        LayerReport r;
        diag(r);
        if(r.bounded<LayerStatus::Unbounded>()){ // valid
            stat.summary(r.get(), r.reason());
            for(std::vector<std::pair<std::string, std::string> >::const_iterator it = r.values().begin(); it != r.values().end(); ++it){
                stat.add(it->first, it->second);
            }
        }
        // for(size_t i=0; i < entries_.size(); ++i) entries_[i](stat); TODO
    }
    virtual ~Logger() {}
};

template<typename InterfaceType> class RosChain : public canopen::LayerStack {
protected:
    std::string chain_name_;
    
    boost::shared_ptr<InterfaceType> interface_;
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

    boost::weak_ptr<LayerStatus> pending_status_;
    
    void logState(const can::State &s){
        boost::shared_ptr<InterfaceType> interface = interface_;
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
                boost::shared_ptr<LayerStatus> pending_status = pending_status_.lock();
                if(pending_status) pending(*pending_status);
                write(s);
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
        boost::shared_ptr<LayerStatus> pending_status(new LayerStatus);
        pending_status_ = pending_status;
        try{
            init(*pending_status);
            res.success.data = pending_status->bounded<LayerStatus::Ok>();
            res.error_message.data = pending_status->reason();
            if(!pending_status->bounded<LayerStatus::Warn>()){
                shutdown(*pending_status);
                thread_.reset();
            }
        }
        catch( const canopen::Exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            res.success.data = false;
            res.error_message.data = info;
            pending_status->error(info);
            shutdown(*pending_status);
            thread_.reset();
        }
        return true;
    }
    virtual bool handle_recover(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
	ROS_INFO("Recovering XXX");
        boost::mutex::scoped_lock lock(mutex_);
        if(thread_){
            boost::shared_ptr<LayerStatus> pending_status(new LayerStatus);
            pending_status_ = pending_status;
            try{
                thread_->interrupt();
                thread_->join();
                thread_.reset(new boost::thread(&RosChain::run, this));
                recover(*pending_status);
                res.success.data = pending_status->bounded<LayerStatus::Warn>();
                res.error_message.data = pending_status->reason();
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
    virtual void shutdown(LayerStatus &status){
        if(thread_){
            halt(status);
            thread_->interrupt();
            thread_->join();
            LayerStack::shutdown(status);
            thread_.reset();
        }
    }

    virtual bool handle_shutdown(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
	ROS_INFO("Shuting down XXX");
        boost::mutex::scoped_lock lock(mutex_);
        if(thread_){
            LayerStatus s;
            shutdown(s);
            res.success.data = s.bounded<LayerStatus::Warn>();
        }else{
            res.success.data = false;
            res.error_message.data = "not running";
            
        }
        return true;
    }
    virtual bool handle_halt(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
	ROS_INFO("Halting down XXX");
        boost::mutex::scoped_lock lock(mutex_);
        if(thread_){
            LayerStatus s;
            halt(s);
            res.success.data = s.bounded<LayerStatus::Warn>();
        }else{
            res.success.data = false;
            res.error_message.data = "not running";

        }
        return true;
    }
    bool setup_bus(){
        ros::NodeHandle bus_nh(nh_priv_,"bus");
        std::string can_device;
        std::string master_type;
        int can_bitrate = 0;
        
        if(!bus_nh.getParam("device",can_device)){
            ROS_ERROR("Device not set");
            return false;
        }
        
        bus_nh.param("bitrate",can_bitrate, 0);
        
        if(can_bitrate < 0){
            ROS_ERROR_STREAM("CAN bitrate  "<< can_bitrate << " is invalid");
            return false;
        }
        interface_ = boost::make_shared<InterfaceType>(true); // enable loopback
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
        
        add(boost::make_shared<CANLayer<InterfaceType> >(interface_, can_device, can_bitrate));
        
        return true;
    }
    bool setup_sync(){
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

        if(!sync_nh.getParam("overflow", sync_overflow)){
            ROS_WARN("Sync overflow was not specified, so overflow is disabled per default");
        }
        if(sync_overflow == 1 || sync_overflow > 240){
            ROS_ERROR_STREAM("Sync overflow  "<< sync_overflow << " is invalid");
            return false;
        }

        if(sync_ms){
            // TODO: parse header
            sync_ = master_->getSync(SyncProperties(can::MsgHeader(0x80), boost::posix_time::milliseconds(sync_ms), sync_overflow));
            
            if(!sync_ && sync_ms){
                ROS_ERROR_STREAM("Initializing sync master failed");
                return false;
            }
            add(sync_);
        }
        return true;
    }
    bool setup_nodes(){
        nodes_.reset(new canopen::LayerGroupNoDiag<canopen::Node>("301 layer"));
        add(nodes_);

        XmlRpc::XmlRpcValue modules;
        nh_priv_.getParam("modules", modules);
        MergedXmlRpcStruct defaults;
        nh_priv_.getParam("defaults", defaults);

        for (int32_t i = 0; i < modules.size(); ++i){
            XmlRpc::XmlRpcValue &module = modules[i];
            std::string name = module["name"];
            int node_id;
            try{
                node_id = module["id"];
            }
            catch(...){
                ROS_ERROR_STREAM("Module at list index " << i << " has no id");
                return false;
            }

            MergedXmlRpcStruct merged(module, defaults);
                            
            std::string eds;
            
            try{
                eds = (std::string) merged["eds_file"];
            }
            catch(...){
                ROS_ERROR_STREAM("EDS path '" << eds << "' invalid");
                return false;
            }

            try{
                std::string pkg = merged["eds_pkg"];
                std::string p = ros::package::getPath(pkg);
                if(p.empty()){
                        ROS_ERROR_STREAM("Package '" << pkg << "' not found");
                        return false;
                }
                eds = (boost::filesystem::path(p)/eds).make_preferred().native();;
            }
            catch(...){
            }
            
            boost::shared_ptr<ObjectDict>  dict = ObjectDict::fromFile(eds);
            if(!dict){
                ROS_ERROR_STREAM("EDS '" << eds << "' could not be parsed");
                return false;
            }
            boost::shared_ptr<canopen::Node> node = boost::make_shared<canopen::Node>(interface_, dict, node_id, sync_);

            boost::shared_ptr<Logger> logger = boost::make_shared<Logger>(node);

            if(!nodeAdded(merged, node, logger)) return false;

            //logger->add(4,"pos", canopen::ObjectDict::Key(0x6064));
            loggers_.push_back(logger);
            diag_updater_.add(name, boost::bind(&Logger::log, logger, _1));
            
            nodes_->add(node);
        }
        return true;
    }
    virtual bool nodeAdded(XmlRpc::XmlRpcValue &module, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger) { return true; }
    void report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
        LayerReport r;
        diag(r);
        if(r.bounded<LayerStatus::Unbounded>()){ // valid
            stat.summary(r.get(), r.reason());
            for(std::vector<std::pair<std::string, std::string> >::const_iterator it = r.values().begin(); it != r.values().end(); ++it){
                stat.add(it->first, it->second);
            }
        }
    }
public:
    RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): LayerStack("ROS stack"),nh_(nh), nh_priv_(nh_priv), diag_updater_(nh_,nh_priv_){}
    virtual bool setup(){
        boost::mutex::scoped_lock lock(mutex_);

        if(!nh_priv_.getParam("name", chain_name_)){
            ROS_ERROR("Chain name not set");
            return false;
        }
        std::string hw_id;
        nh_priv_.param("hardware_id", hw_id, std::string("none"));
        
        diag_updater_.setHardwareID(hw_id);
        diag_updater_.add(chain_name_, this, &RosChain::report_diagnostics);
        
        diag_timer_ = nh_.createTimer(ros::Duration(diag_updater_.getPeriod()/2.0),boost::bind(&diagnostic_updater::Updater::update, &diag_updater_));
        
        ros::NodeHandle nh_driver(nh_, "driver");
        
        srv_init_ = nh_driver.advertiseService("init",&RosChain::handle_init, this);
        srv_recover_ = nh_driver.advertiseService("recover",&RosChain::handle_recover, this);
        srv_halt_ = nh_driver.advertiseService("halt",&RosChain::handle_halt, this);
        srv_shutdown_ = nh_driver.advertiseService("shutdown",&RosChain::handle_shutdown, this);
        
        return setup_bus() && setup_sync() && setup_nodes();
    }
    virtual ~RosChain(){
        try{
            LayerStatus s;
            shutdown(s);
        }catch(...){ }
    }
};

} //namespace canopen

#endif

