#ifndef H_CANOPEN_ROS_CHAIN
#define H_CANOPEN_ROS_CHAIN

#include <canopen_master/canopen.h>
#include <canopen_master/can_layer.h>
#include <canopen_chain_node/GetObject.h>
#include <canopen_chain_node/SetObject.h>
#include <socketcan_interface/string.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/filesystem/path.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <pluginlib/class_loader.h>

namespace canopen{

class PublishFunc{
public:
    typedef boost::function<void()> func_type;

    static func_type create(ros::NodeHandle &nh,  const std::string &name, boost::shared_ptr<canopen::Node> node, const std::string &key, bool force);
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

    static void log_entry(diagnostic_updater::DiagnosticStatusWrapper &stat, uint8_t level, const std::string &name, boost::function<std::string()> getter){
        if(stat.level >= level){
            try{
                stat.add(name, getter());
            }catch(...){
                stat.add(name, "<ERROR>");
            }
        }
    }

public:
    Logger(boost::shared_ptr<canopen::Node> node):  node_(node) { add(node_); }

    bool add(uint8_t level, const std::string &key, bool forced){
        try{
            ObjectDict::Key k(key);
            const boost::shared_ptr<const ObjectDict::Entry> entry = node_->getStorage()->dict_->get(k);
            std::string name = entry->desc.empty() ? key : entry->desc;
            entries_.push_back(boost::bind(log_entry, _1, level, name, node_->getStorage()->getStringReader(k, !forced)));
            return true;
        }
        catch(std::exception& e){
            ROS_ERROR_STREAM(boost::diagnostic_information(e));
            return false;
        }
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
                for(size_t i=0; i < entries_.size(); ++i) entries_[i](stat);
            }
        }
    }
    virtual ~Logger() {}
};

class GuardedClassLoaderList {
    static std::vector< boost::shared_ptr<pluginlib::ClassLoaderBase> >& guarded_loaders(){
        static std::vector< boost::shared_ptr<pluginlib::ClassLoaderBase> > loaders;
        return loaders;
    }
public:
    static void addLoader(boost::shared_ptr<pluginlib::ClassLoaderBase> b){
        guarded_loaders().push_back(b);
    }
   ~GuardedClassLoaderList(){
       guarded_loaders().clear();
   }
};

template<typename T> class GuardedClassLoader {
    typedef pluginlib::ClassLoader<T> Loader;
    boost::shared_ptr<Loader> loader_;
public:
    GuardedClassLoader(const std::string& package, const std::string& allocator_base_class)
    : loader_(new Loader(package, allocator_base_class)) {
        GuardedClassLoaderList::addLoader(loader_);
    }
    boost::shared_ptr<T> createInstance(const std::string& lookup_name){
        return loader_->createInstance(lookup_name);
    }
};

template<typename T> class ClassAllocator : public GuardedClassLoader<typename T::Allocator> {
public:
    ClassAllocator (const std::string& package, const std::string& allocator_base_class) : GuardedClassLoader<typename T::Allocator>(package, allocator_base_class) {}
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
class RosChain : GuardedClassLoaderList, public canopen::LayerStack {
    GuardedClassLoader<can::DriverInterface> driver_loader_;
    ClassAllocator<canopen::Master> master_allocator_;
protected:
    boost::shared_ptr<can::DriverInterface> interface_;
    boost::shared_ptr<Master> master_;
    boost::shared_ptr<canopen::LayerGroupNoDiag<canopen::Node> > nodes_;
    boost::shared_ptr<canopen::LayerGroupNoDiag<canopen::EMCYHandler> > emcy_handlers_;
    std::map<std::string, boost::shared_ptr<canopen::Node> > nodes_lookup_;
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
    ros::ServiceServer srv_get_object_;
    ros::ServiceServer srv_set_object_;

    time_duration update_duration_;

    struct HeartbeatSender{
      can::Frame frame;
      boost::shared_ptr<can::DriverInterface> interface;
      bool send(){
          return interface && interface->send(frame);
      }
    } hb_sender_;
    Timer heartbeat_timer_;

    boost::atomic<bool> running_;
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
    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger);
    void report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual bool setup_chain();
public:
    RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
    bool setup();
    virtual ~RosChain();
};

} //namespace canopen

#endif

