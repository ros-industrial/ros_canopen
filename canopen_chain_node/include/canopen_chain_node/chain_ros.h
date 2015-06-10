#ifndef H_CANOPEN_CHAIN_ROS
#define H_CANOPEN_CHAIN_ROS

#include <canopen_master/canopen.h>
#include <canopen_master/can_layer.h>
#include <socketcan_interface/string.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/filesystem/path.hpp>
#include <boost/weak_ptr.hpp>
#include <pluginlib/class_loader.h>

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

namespace canopen{

class PublishFunc{
public:
    typedef boost::function<void()> func_type;

    static func_type create(ros::NodeHandle &nh,  const std::string &name, boost::shared_ptr<canopen::Node> node, const std::string &key, bool force){
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
private:
    template <typename Tpub, typename Tobj, bool forced> static void publish(ros::Publisher &pub, ObjectStorage::Entry<Tobj> &entry){
		Tpub msg;
		msg.data = (const typename Tpub::_data_type &)(forced? entry.get() : entry.get_cached());
        pub.publish(msg);
    }
    template<typename Tpub, typename Tobj> static func_type create(ros::NodeHandle &nh,  const std::string &name, ObjectStorage::Entry<Tobj> entry, bool force){
        if(!entry.valid()) return 0;
        ros::Publisher pub = nh.advertise<Tpub>(name, 1);
        if(force){
            return boost::bind(PublishFunc::publish<Tpub, Tobj, true>, pub, entry);
        }else{
            return boost::bind(PublishFunc::publish<Tpub, Tobj, false>, pub, entry);
        }
    }
};

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

template<typename T> class ClassAllocator : public pluginlib::ClassLoader<typename T::Allocator> {
public:
    ClassAllocator (const std::string& package, const std::string& allocator_base_class) : pluginlib::ClassLoader<typename T::Allocator>(package, allocator_base_class) {}
    template<typename T1> boost::shared_ptr<T> allocateInstance(const std::string& lookup_name, const T1 & t1){
        return this->createInstance(lookup_name)->allocate(t1);
    }
    template<typename T1, typename T2> boost::shared_ptr<T> allocateInstance(const std::string& lookup_name, const T1 & t1, const T2 & t2){
        return this->createInstance(lookup_name)->allocate(t1, t2);
    }
    template<typename T1, typename T2, typename T3> boost::shared_ptr<T> allocateInstance(const std::string& lookup_name, const T1 & t1, const T2 & t2, const T3 & t3){
        return this->createInstance(lookup_name)->allocate(t1, t2, t3);
    }
};

class RosChain : public canopen::LayerStack {
      pluginlib::ClassLoader<can::DriverInterface> driver_loader_;
      ClassAllocator<canopen::Master> master_allocator_;
protected:
    boost::shared_ptr<can::DriverInterface> interface_;
    boost::shared_ptr<Master> master_;
    boost::shared_ptr<canopen::LayerGroupNoDiag<canopen::Node> > nodes_;
    boost::shared_ptr<canopen::SyncLayer> sync_;
    std::vector<boost::shared_ptr<Logger > > loggers_;
    std::vector<PublishFunc::func_type> publishers_;

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

    Timer heartbeat_timer_;

    boost::atomic<bool> initialized_;
    boost::mutex diag_mutex_;

    void logState(const can::State &s){
        boost::shared_ptr<can::DriverInterface> interface = interface_;
        std::string msg;
        if(interface && !interface->translateError(s.internal_error, msg)) msg  =  "Undefined"; ;
        ROS_INFO_STREAM("Current state: " << s.driver_state << " device error: " << s.error_code << " internal_error: " << s.internal_error << " (" << msg << ")");
    }
    
    void run(){

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
    
    virtual bool handle_init(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
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
                shutdown(status);
                initialized_ = false;
                thread_.reset();
            }else{
                heartbeat_timer_.restart();
            }
        }
        catch( const canopen::Exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            res.success = false;
            res.message = info;
            status.error(info);
            shutdown(status);
            thread_.reset();
        }
        return true;
    }
    virtual bool handle_recover(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
	ROS_INFO("Recovering XXX");
        boost::mutex::scoped_lock lock(mutex_);
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
            catch( const canopen::Exception &e){
                std::string info = boost::diagnostic_information(e);
                ROS_ERROR_STREAM(info);
                res.success = false;
                res.message = info;
            }
        }else{
            res.success = false;
            res.message = "not running";
        }
        return true;
    }

    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        LayerStack::handleWrite(status, current_state);
        if(current_state > Init){
            for(std::vector<boost::function<void() > >::iterator it = publishers_.begin(); it != publishers_.end(); ++it) (*it)();
        }
    }
    virtual void handleShutdown(LayerStatus &status){
        boost::mutex::scoped_lock lock(diag_mutex_);
        LayerStack::handleShutdown(status);
        heartbeat_timer_.stop();
        if(initialized_ &&  thread_){
            thread_->interrupt();
            thread_->join();
            thread_.reset();
        }
        initialized_ = false;
    }

    virtual bool handle_shutdown(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
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
    virtual bool handle_halt(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
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
    bool setup_bus(){
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

        bus_nh.param("master_allocator",master_alloc, std::string("canopen::LocalMaster::Allocator"));

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
    struct HeartbeatSender{
      can::Frame frame;
      boost::shared_ptr<can::DriverInterface> interface;
      bool send(){
          return interface && interface->send(frame);
      }
    } hb_sender_;

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

            hb_sender_.frame = can::toframe(msg);


            if(!hb_sender_.frame.isValid()){
                ROS_ERROR_STREAM("Message '"<< msg << "' is invalid");
                return false;
            }

            hb_sender_.interface = interface_;

            heartbeat_timer_.start(Timer::TimerDelegate(&hb_sender_, &HeartbeatSender::send) , boost::chrono::duration<double>(1.0/rate), false);

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
    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger) { return true; }

    void report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
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
public:
    RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
    : LayerStack("ROS stack"),driver_loader_("socketcan_interface", "can::DriverInterface"), master_allocator_("canopen_master", "canopen::Master::Allocator"),nh_(nh), nh_priv_(nh_priv), diag_updater_(nh_,nh_priv_), initialized_(false){}
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
        publishers_.clear();
        try{
            LayerStatus s;
            halt(s);
            shutdown(s);
        }catch(...){ LOG("CATCH"); }
        destroy();
    }
};

} //namespace canopen

#endif

