#ifndef H_IPA_CANOPEN_CHAIN_ROS
#define H_IPA_CANOPEN_CHAIN_ROS

#include <ipa_canopen_master/canopen.h>
#include <ipa_canopen_master/master.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cob_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/filesystem/path.hpp>
#include <boost/weak_ptr.hpp>

namespace ipa_canopen{
    
template<typename T> bool read_xmlrpc_or_praram(T &val, const std::string &name, XmlRpc::XmlRpcValue &valstruct, const ros::NodeHandle &nh){
    if(valstruct.hasMember(name)){
        val = static_cast<T>(valstruct[name]);
        return true;
    }
    return nh.getParam(name, val);
}

template<typename NodeType> class Logger{
    boost::weak_ptr<NodeType> node_;
    std::vector<boost::function< void (diagnostic_updater::DiagnosticStatusWrapper &)> > entries_;
    
    template<typename T> void log_entry(diagnostic_updater::DiagnosticStatusWrapper &stat, const std::string &name, const ObjectDict::Key &key){
        boost::shared_ptr<NodeType> node = node_.lock();
        if(node){
            stat.add(name, node->template get<T>(key));
        }
    }
public:
    Logger(boost::shared_ptr<NodeType> node):  node_(node) {}
    template<typename T> void add(const std::string &name, const ObjectDict::Key &key){
            entries_.push_back(boost::bind(&Logger::log_entry<T>, this, _1, name, key));
    }
    template<const uint16_t dt> static void func(Logger &l, const std::string &n, const ObjectDict::Key &k){
        l.template add<typename ObjectStorage::DataType<dt>::type>(n,k);
    }
    void add(const uint16_t data_type, const std::string &name, const ObjectDict::Key &key){
        branch_type<Logger, void (Logger &, const std::string &, const ObjectDict::Key &)>(data_type)(*this,name, key);
    }

    virtual void log(diagnostic_updater::DiagnosticStatusWrapper &stat){
        boost::shared_ptr<NodeType> node = node_.lock();
        if(node){
            stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active");
            if(node->getState() == ipa_canopen::Node::Operational)
                for(size_t i=0; i < entries_.size(); ++i) entries_[i](stat);
            //ROS_INFO_STREAM(node->getStorage()->template entry<std::string>(0x1008).get_once());
            //stat.add("desc",std::string());

        }
    }
    virtual ~Logger() {}
};

template<typename InterfaceType, typename MasterType> class RosChain : public ipa_canopen::LayerStack {
protected:
    std::string chain_name_;
    
    boost::shared_ptr<InterfaceType> interface_;
    boost::shared_ptr<MasterType> master_;
    boost::shared_ptr<ipa_canopen::LayerGroup<ipa_canopen::Node> > nodes_;
    boost::shared_ptr<ipa_canopen::SyncLayer> sync_;

    ipa_can::StateInterface::StateListener::Ptr state_listener_;
    
    boost::scoped_ptr<boost::thread> thread_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    
    diagnostic_updater::Updater diag_updater_;
    ros::Timer diag_timer_;

    boost::mutex mutex_;
    ros::ServiceServer srv_init_;
    ros::ServiceServer srv_recover_;
    ros::ServiceServer srv_shutdown_;
    
    void logState(const ipa_can::State &s){
        boost::shared_ptr<InterfaceType> interface = interface_;
        std::string msg =  "Undefined";
        if(interface) interface->translateError(s.internal_error, msg);
        ROS_INFO_STREAM("Current state: " << s.driver_state << " device error: " << s.error_code << " internal_error: " << s.internal_error << " (" << msg << ")");
    }
    
    void run(){

        LayerStatus s;
        if(sync_){
            sync_->read(s);
            s.reset();
            sync_->write(s);
        }
        while(ros::ok()){
            s.reset();
            read(s);
            s.reset();
            write(s);
            boost::this_thread::interruption_point();
        }
    }
    
    virtual bool handle_init(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
        boost::mutex::scoped_lock lock(mutex_);
        LayerStatusExtended s;
        init(s);
        res.success.data = s.bounded(LayerStatus::WARN);
        if(res.success.data){
            thread_.reset(new boost::thread(&RosChain::run, this));
        }
        return true;
    }
    virtual bool handle_recover(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
        boost::mutex::scoped_lock lock(mutex_);
        LayerStatusExtended s;
        recover(s);
        res.success.data = s.bounded(LayerStatus::WARN);
        return true;
    }
    virtual bool handle_shutdown(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
        boost::mutex::scoped_lock lock(mutex_);
        if(thread_){
            thread_->interrupt();
            thread_->join();
        }
        thread_.reset();
        LayerStatus s;
        shutdown(s);
        res.success.data = s.bounded(LayerStatus::WARN);
        return true;
    }
    bool setup_bus(){
        ros::NodeHandle bus_nh(nh_priv_,"bus");
        std::string can_device;
        int can_bitrate = 0;
        
        if(!bus_nh.getParam("device",can_device)){
            ROS_ERROR("Device not set");
            return false;
        }
        
        bool can_loopback;

        bus_nh.param("bitrate",can_bitrate, 0);
        
        if(can_bitrate < 0){
            ROS_ERROR_STREAM("CAN bitrate  "<< can_bitrate << " is invalid");
            return false;
        }
        
        bus_nh.param("loopback",can_loopback, true);

        interface_ = boost::make_shared<InterfaceType>(can_loopback);
        state_listener_ = interface_->createStateListener(ipa_can::StateInterface::StateDelegate(this, &RosChain::logState));
        
        master_ = boost::make_shared<MasterType>(interface_);
        
        add(boost::make_shared<CANLayer<InterfaceType> >(interface_, can_device, can_bitrate));
        
        return true;
    }
    bool setup_sync(){
        ros::NodeHandle sync_nh(nh_priv_,"sync");
        
        //TODO: fallback to update_interval
        
        int sync_ms = 0;
        int sync_overflow = 0;
        
        if(!sync_nh.getParam("interval_ms", sync_ms)){
            ROS_WARN("Sync interval was not specified, so sync is disabled per default");
        }
        
        if(sync_ms < 0){
            ROS_ERROR_STREAM("Sync interval  "<< sync_ms << " is invalid");
            return false;
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
            sync_ = master_->getSync(ipa_can::Header(0x80), boost::posix_time::milliseconds(sync_ms), sync_overflow);
            
            if(!sync_ && sync_ms){
                ROS_ERROR_STREAM("Initializing sync master failed");
                return false;
            }
            add(sync_);
        }
        return true;
    }
    bool setup_nodes(){
        nodes_.reset(new ipa_canopen::LayerGroup<ipa_canopen::Node>);
        add(nodes_);

        XmlRpc::XmlRpcValue modules;
        nh_priv_.getParam("modules", modules);
        ros::NodeHandle def_nh(nh_priv_,"defaults");

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
            std::string eds;
            std::string pkg;
            
            if(!read_xmlrpc_or_praram(eds, "eds_file", module, def_nh) || eds.empty()){
                ROS_ERROR_STREAM("EDS path '" << eds << "' invalid");
                return false;
            }
            if(read_xmlrpc_or_praram(pkg, "eds_pkg", module, def_nh)){
            std::string p = ros::package::getPath(pkg);
            if(p.empty()){
                    ROS_ERROR_STREAM("Package '" << pkg << "' not found");
                    return false;
            }
            eds = (boost::filesystem::path(p)/eds).make_preferred().native();;
            }
            boost::shared_ptr<ObjectDict>  dict = ObjectDict::fromFile(eds);
            if(!dict){
                ROS_ERROR_STREAM("EDS '" << eds << "' could not be parsed");
                return false;
            }
            boost::shared_ptr<ipa_canopen::Node> node = boost::make_shared<ipa_canopen::Node>(interface_, dict, node_id, sync_);
            
            if(!nodeAdded(module, node)) return false;

            boost::shared_ptr<Logger<ipa_canopen::Node> > logger = boost::make_shared<Logger<ipa_canopen::Node> >(node);
            //logger->add(4,"pos", ipa_canopen::ObjectDict::Key(0x6064));
            diag_updater_.add(name, boost::bind(&Logger<Node>::log, logger, _1));
            
            nodes_->add(node);
        }
        return true;
    }
    virtual bool nodeAdded(XmlRpc::XmlRpcValue &module, const boost::shared_ptr<ipa_canopen::Node> &node) { return true; }
public:
    RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): nh_(nh), nh_priv_(nh_priv), diag_updater_(nh_,nh_priv_){}
    virtual bool setup(){
        boost::mutex::scoped_lock lock(mutex_);

        if(!nh_priv_.getParam("name", chain_name_)){
            ROS_ERROR("Chain name not set");
            return false;
        }
        std::string hw_id;
        nh_priv_.param("hardware_id", hw_id, std::string("none"));
        
        diag_updater_.setHardwareID(hw_id);
        diag_timer_ = nh_.createTimer(ros::Duration(diag_updater_.getPeriod()/2.0),boost::bind(&diagnostic_updater::Updater::update, &diag_updater_));
        
        ros::NodeHandle nh_chain(nh_, chain_name_);
        
        srv_init_ = nh_chain.advertiseService("init",&RosChain::handle_init, this);
        srv_recover_ = nh_chain.advertiseService("recover",&RosChain::handle_recover, this);
        srv_shutdown_ = nh_chain.advertiseService("shutdown",&RosChain::handle_shutdown, this);
        
        return setup_bus() && setup_sync() && setup_nodes();
    }
    virtual ~RosChain(){
        LayerStatus s;
        shutdown(s);
    }
};

} //namespace ipa_canopen

#endif

