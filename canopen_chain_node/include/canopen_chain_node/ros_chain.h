#ifndef H_CANOPEN_ROS_CHAIN
#define H_CANOPEN_ROS_CHAIN

#include <memory>
#include <canopen_master/canopen.h>
#include <canopen_master/can_layer.h>
#include <canopen_chain_node/GetObject.h>
#include <canopen_chain_node/SetObject.h>
#include <socketcan_interface/string.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/filesystem/path.hpp>
#include <pluginlib/class_loader.hpp>

namespace canopen{

typedef std::function<void()> PublishFuncType;
PublishFuncType createPublishFunc(ros::NodeHandle &nh,  const std::string &name, canopen::NodeSharedPtr node, const std::string &key, bool force);

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
    const canopen::NodeSharedPtr node_;

    std::vector<std::function< void (diagnostic_updater::DiagnosticStatusWrapper &)> > entries_;

    static void log_entry(diagnostic_updater::DiagnosticStatusWrapper &stat, uint8_t level, const std::string &name, std::function<std::string()> getter){
        if(stat.level >= level){
            try{
                stat.add(name, getter());
            }catch(...){
                stat.add(name, "<ERROR>");
            }
        }
    }

public:
    Logger(canopen::NodeSharedPtr node):  node_(node) { add(node_); }

    bool add(uint8_t level, const std::string &key, bool forced){
        try{
            ObjectDict::Key k(key);
            const ObjectDict::EntryConstSharedPtr entry = node_->getStorage()->dict_->get(k);
            std::string name = entry->desc.empty() ? key : entry->desc;
            entries_.push_back(std::bind(log_entry, std::placeholders::_1, level, name, node_->getStorage()->getStringReader(k, !forced)));
            return true;
        }
        catch(std::exception& e){
            ROS_ERROR_STREAM(boost::diagnostic_information(e));
            return false;
        }
    }

    template<typename T> void add(const std::shared_ptr<T> &n){
        DiagGroup::add(std::static_pointer_cast<canopen::Layer>(n));
    }

    virtual void log(diagnostic_updater::DiagnosticStatusWrapper &stat){
        if(node_->getState() == canopen::Node::Unknown){
            stat.summary(stat.WARN,"Not initialized");
        }else{
            LayerReport r;
            diag(r);
            if(r.bounded<LayerStatus::Unbounded>()){ // valid
                stat.summary(r.get(), r.reason());
                for(std::vector<std::pair<std::string, std::string> >::const_iterator it = r.values().begin(); it != r.values().end(); ++it){
                    stat.add(it->first, it->second);
                }
                for(size_t i=0; i < entries_.size(); ++i) entries_[i](stat);
            }
        }
    }
    virtual ~Logger() {}
};
typedef std::shared_ptr<Logger> LoggerSharedPtr;

class GuardedClassLoaderList {
public:
    typedef std::shared_ptr<pluginlib::ClassLoaderBase> ClassLoaderBaseSharedPtr;
    static void addLoader(ClassLoaderBaseSharedPtr b){
        guarded_loaders().push_back(b);
    }
   ~GuardedClassLoaderList(){
       guarded_loaders().clear();
   }
private:
   static std::vector< ClassLoaderBaseSharedPtr>& guarded_loaders(){
       static std::vector<ClassLoaderBaseSharedPtr> loaders;
       return loaders;
   }
};

template<typename T> class GuardedClassLoader {
    typedef pluginlib::ClassLoader<T> Loader;
    std::shared_ptr<Loader> loader_;
public:
    typedef std::shared_ptr<T> ClassSharedPtr;
    GuardedClassLoader(const std::string& package, const std::string& allocator_base_class)
    : loader_(new Loader(package, allocator_base_class)) {
        GuardedClassLoaderList::addLoader(loader_);
    }
    ClassSharedPtr createInstance(const std::string& lookup_name){
        return loader_->createUniqueInstance(lookup_name);
    }
};

template<typename T> class ClassAllocator : public GuardedClassLoader<typename T::Allocator> {
public:
    typedef std::shared_ptr<T> ClassSharedPtr;
    ClassAllocator (const std::string& package, const std::string& allocator_base_class) : GuardedClassLoader<typename T::Allocator>(package, allocator_base_class) {}
    template<typename T1> ClassSharedPtr allocateInstance(const std::string& lookup_name, const T1 & t1){
        return this->createInstance(lookup_name)->allocate(t1);
    }
    template<typename T1, typename T2> ClassSharedPtr allocateInstance(const std::string& lookup_name, const T1 & t1, const T2 & t2){
        return this->createInstance(lookup_name)->allocate(t1, t2);
    }
    template<typename T1, typename T2, typename T3> ClassSharedPtr allocateInstance(const std::string& lookup_name, const T1 & t1, const T2 & t2, const T3 & t3){
        return this->createInstance(lookup_name)->allocate(t1, t2, t3);
    }
};
class RosChain : GuardedClassLoaderList, public canopen::LayerStack {
private:
    GuardedClassLoader<can::DriverInterface> driver_loader_;
    ClassAllocator<canopen::Master> master_allocator_;
    bool setup_node(const XmlRpc::XmlRpcValue &params, const std::string& name, const MergedXmlRpcStruct &defaults);
protected:
    can::DriverInterfaceSharedPtr interface_;
    MasterSharedPtr master_;
    std::shared_ptr<canopen::LayerGroupNoDiag<canopen::Node> > nodes_;
    std::shared_ptr<canopen::LayerGroupNoDiag<canopen::EMCYHandler> > emcy_handlers_;
    std::map<std::string, canopen::NodeSharedPtr > nodes_lookup_;
    canopen::SyncLayerSharedPtr sync_;
    std::vector<LoggerSharedPtr > loggers_;
    std::vector<PublishFuncType> publishers_;

    can::StateListenerConstSharedPtr state_listener_;

    std::unique_ptr<boost::thread> thread_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    diagnostic_updater::Updater diag_updater_;
    ros::Timer diag_timer_;

    boost::mutex mutex_;
    ros::ServiceServer srv_init_;
    ros::ServiceServer srv_recover_;
    ros::ServiceServer srv_halt_;
    ros::ServiceServer srv_shutdown_;
    ros::ServiceServer srv_get_object_;
    ros::ServiceServer srv_set_object_;

    time_duration update_duration_;

    struct HeartbeatSender{
      can::Frame frame;
      can::DriverInterfaceSharedPtr interface;
      bool send(){
          return interface && interface->send(frame);
      }
    } hb_sender_;
    Timer heartbeat_timer_;

    std::atomic<bool> running_;
    boost::mutex diag_mutex_;

    bool reset_errors_before_recover_;

    void logState(const can::State &s);
    void run();
    virtual bool handle_init(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
    virtual bool handle_recover(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleShutdown(LayerStatus &status);
    virtual bool handle_shutdown(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
    virtual bool handle_halt(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

    bool handle_get_object(canopen_chain_node::GetObject::Request  &req, canopen_chain_node::GetObject::Response &res);
    bool handle_set_object(canopen_chain_node::SetObject::Request  &req, canopen_chain_node::SetObject::Response &res);

    bool setup_bus();
    bool setup_sync();
    bool setup_heartbeat();
    bool setup_nodes();
    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const canopen::NodeSharedPtr &node, const LoggerSharedPtr &logger);
    void report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual bool setup_chain();
public:
    RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
    bool setup();
    virtual ~RosChain();
};

} //namespace canopen

#endif
