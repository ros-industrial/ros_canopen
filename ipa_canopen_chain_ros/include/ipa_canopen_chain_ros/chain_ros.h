#ifndef H_IPA_CANOPEN_CHAIN_ROS
#define H_IPA_CANOPEN_CHAIN_ROS

#include <ipa_canopen_master/canopen.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cob_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/filesystem/path.hpp>

namespace ipa_canopen{

class StateWaiter{
    boost::mutex mutex_;
    boost::condition_variable cond_;
    ipa_can::Interface::StateListener::Ptr state_listener_;
    ipa_can::State state_;
    void updateState(const ipa_can::State &s){
        boost::mutex::scoped_lock lock(mutex_);
        state_ = s;
        lock.unlock();
        cond_.notify_one();
    }
public:
    template<typename InterfaceType> StateWaiter(boost::shared_ptr<InterfaceType> interface){
        state_ = interface->getState();
        state_listener_ = interface->createStateListener(ipa_can::Interface::StateDelegate(this, &StateWaiter::updateState));
    }
    template<typename DurationType> bool wait(const ipa_can::State::DriverState &s, const DurationType &duration){
        boost::mutex::scoped_lock cond_lock(mutex_);
        boost::system_time abs_time = boost::get_system_time() + duration;
        while(s != state_.driver_state)
        {
            if(!cond_.timed_wait(cond_lock,abs_time))
            {
                return false;
            }
        }
        return true;
    }
    template<typename InterfaceType, typename DurationType> static bool wait_for(const ipa_can::State::DriverState &s, boost::shared_ptr<InterfaceType> interface, const DurationType &duration){
        StateWaiter waiter(interface);
        return waiter.wait(s,duration);
    }
};

template<typename T> bool read_xmlrpc_or_praram(T &val, const std::string &name, XmlRpc::XmlRpcValue &valstruct, const ros::NodeHandle &nh){
    if(valstruct.hasMember(name)){
        val = static_cast<T>(valstruct[name]);
        return true;
    }
    return nh.getParam(name, val);
}

template<typename NodeType, typename InterfaceType, typename MasterType> class RosChain {
protected:
    
    std::string chain_name_;
    
    boost::shared_ptr<InterfaceType> interface_;
    boost::shared_ptr<MasterType> master_;
    boost::scoped_ptr<ipa_canopen::NodeChain<NodeType> > nodes_;
    ipa_can::Interface::StateListener::Ptr state_listener_;
    
    boost::scoped_ptr<boost::thread> thread_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    
    diagnostic_updater::Updater diag_updater_;
    ros::Timer diag_timer_;
    
    boost::mutex mutex_;
    ros::ServiceServer srv_init_;
    ros::ServiceServer srv_halt_;
    ros::ServiceServer srv_recover_;
    ros::ServiceServer srv_shutdown_;
    
    void logState(const ipa_can::State &s){
        boost::shared_ptr<InterfaceType> interface = interface_;
        std::string msg =  "Undefined";
        if(interface) interface->translateError(s.internal_error, msg);
        ROS_INFO_STREAM("Current state: " << s.driver_state << " device error: " << s.error_code << " internal_error: " << s.internal_error << " (" << msg << ")");
    }
    
    void _destroy(){
        state_listener_.reset();
        if(thread_){
            interface_->shutdown();
            thread_->join();
        }
        thread_.reset();
        nodes_.reset();
        master_.reset();
        interface_.reset();
        
    }
    virtual bool handle_init(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
        boost::mutex::scoped_lock lock(mutex_);
        res.success.data = false;
        
        ros::NodeHandle bus_nh(nh_priv_,"bus");
        std::string can_device;
        int can_bitrate = 0;
        
        if(!bus_nh.getParam("device",can_device)){
            _destroy();
            ROS_ERROR("Device not set");
            return true;
        }
        
        bool can_loopback;

        bus_nh.param("bitrate",can_bitrate, 0);
        
        if(can_bitrate < 0){
            _destroy();
            ROS_ERROR_STREAM("CAN bitrate  "<< can_bitrate << " is invalid");
            return true;
        }
        
        bus_nh.param("loopback",can_loopback, true);

        interface_ = boost::make_shared<InterfaceType>(can_loopback);
        state_listener_ = interface_->createStateListener(ipa_can::Interface::StateDelegate(this, &RosChain::logState));
        
        if(!interface_->init(can_device, can_bitrate)){
            _destroy();
            ROS_ERROR_STREAM("Initializing can device "<< can_device << " failed");
            return true;
        }
        
        thread_.reset(new boost::thread(&InterfaceType::run, interface_));
        
        if(!StateWaiter::wait_for(ipa_can::State::ready, interface_, boost::posix_time::seconds(1))){
            _destroy();
            ROS_ERROR_STREAM("Initializing can device "<< can_device << " failed");
            return true;
        }
        
        master_ = boost::make_shared<MasterType>(interface_);
        
        ros::NodeHandle sync_nh(nh_priv_,"sync");
        boost::shared_ptr<SyncProvider> sync;
        
        int sync_ms = 0;
        int sync_overflow = 0;
        
        if(!sync_nh.getParam("interval_ms", sync_ms)){
            ROS_WARN("Sync interval was not specified, so sync is disabled per default");
        }
        
        if(sync_ms < 0){
            _destroy();
            ROS_ERROR_STREAM("Sync interval  "<< sync_ms << " is invalid");
            return true;
        }
        
        if(!sync_nh.getParam("overflow", sync_overflow)){
            ROS_WARN("Sync overflow was not specified, so overflow is disabled per default");
        }
        if(sync_overflow < 0 || sync_overflow > 255){
            _destroy();
            ROS_ERROR_STREAM("Sync overflow  "<< sync_overflow << " is invalid");
            return true;
        }

        if(sync_ms){
            // TODO: parse header
            sync = master_->getSync(ipa_can::Header(0x80), boost::posix_time::milliseconds(sync_ms), sync_overflow, can_loopback);
            
            if(!sync && sync_ms){
                _destroy();
                ROS_ERROR_STREAM("Initializing sync master "<< can_device << " failed");
                return true;
            }
        }
        
        nodes_.reset(new ipa_canopen::NodeChain<NodeType>);

        XmlRpc::XmlRpcValue modules;
        nh_priv_.getParam("modules", modules);
        ros::NodeHandle def_nh(nh_priv_,"defaults");

        try{
            for (int32_t i = 0; i < modules.size(); ++i){
                XmlRpc::XmlRpcValue &module = modules[i];
                // TODO: name
                int node_id;
                try{
                node_id = module["id"];
                }
                catch(...){
                    _destroy();
                    ROS_ERROR_STREAM("Module at list index " << i << " has no id");
                    return true;
                }
                std::string eds;
                std::string pkg;
                
                if(!read_xmlrpc_or_praram(eds, "eds_file", module, def_nh) || eds.empty()){
                    _destroy();
                    ROS_ERROR_STREAM("EDS path '" << eds << "' invalid");
                    return true;
                }
                if(read_xmlrpc_or_praram(pkg, "eds_pkg", module, def_nh)){
                std::string p = ros::package::getPath(pkg);
                if(p.empty()){
                        _destroy();
                        ROS_ERROR_STREAM("Package '" << pkg << "' not found");
                        return true;
                }
                eds = (boost::filesystem::path(p)/eds).make_preferred().native();;
                }
                boost::shared_ptr<ipa_canopen::ObjectDict>  dict = ipa_canopen::ObjectDict::fromFile(eds);
                if(!dict){
                    _destroy();
                    ROS_ERROR_STREAM("EDS '" << eds << "' could not be parsed");
                    return true;
                }
                nodes_->add(boost::make_shared<NodeType>(interface_, dict, node_id, sync));
            }
            nodes_->start();
        }
        catch(TimeoutException){
           ROS_ERROR("Timeout");
           return true;
        }
        res.success.data = true;
        return true;
    }
    virtual bool handle_halt(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
        boost::mutex::scoped_lock lock(mutex_);
        return true;
    }
    virtual bool handle_recover(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
        boost::mutex::scoped_lock lock(mutex_);
        return true;
    }
    virtual bool handle_shutdown(cob_srvs::Trigger::Request  &req, cob_srvs::Trigger::Response &res){
        boost::mutex::scoped_lock lock(mutex_);
        return true;
    }
public:
    RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): nh_(nh), nh_priv_(nh_priv), diag_updater_(nh_,nh_priv_){}
    bool setup(){
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
        srv_halt_ = nh_chain.advertiseService("halt",&RosChain::handle_halt, this);
        srv_recover_ = nh_chain.advertiseService("recover",&RosChain::handle_recover, this);
        srv_shutdown_ = nh_chain.advertiseService("shutdown",&RosChain::handle_shutdown, this);
        
        return true;
    }
    virtual ~RosChain(){
        diag_timer_.stop();
        _destroy();
    }
};

} //namespace ipa_canopen

#endif

